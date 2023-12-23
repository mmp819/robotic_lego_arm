/*
 * File: main.c
 *
 * Descripcion: Programa que controla un brazo robotico elaborado con LEGO
 *              y el sistema EV3.
 *
 * Author: Mario Martin Perez <mmp819@alumnos.unican.es>
 * Version: 1.0
 * Date: dec-23
 */

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <error_checks.h>
#include <timespec_operations.h>

#include "ev3c.h"

// Puertos de motores
#define LARGE_ROTATION_MOTOR_PORT   'C'
#define LARGE_ELEVATION_MOTOR_PORT  'B'
#define MEDIUM_CLAW_MOTOR_PORT      'A'

// Puertos de sensores
#define COLOR_SENSOR_PORT           1
#define TOUCH_SENSOR_PORT           2

// Velocidades maximas de los motores
#define FULL_SPEED_LARGE_MOTOR      900     // units: deg/seg
#define FULL_SPEED_MEDIUM_MOTOR     1200    // units: deg/seg

// Tiempo para mandar comandos a los motores y comprobar su estado
#define SUSPENSION_TIME             2000    // units: usecs
#define CHECK_STATE_TIME            1000    // units: nsecs

// Numero de botones (ev3 brick)
#define BUTTONS                     6

// Run-Direct Potencia
#define ROTATION_POWER              30
#define ELEVATION_UP_POWER         -30
#define ELEVATION_DOWN_POWER        20
#define CLAW_POWER                  40

// Unidades de movimiento de los motores para alcanzzar posicion inicial
#define ROTATION_INIT_UNITS         -350
#define ELEVATION_INIT_UNITS        100
#define CLAW_INIT_UNITS             90

// Touch Sensor
#define TOUCH_SENSOR_ACTIVE         1
#define TOUCH_SENSOR_INACTIVE       0

// Valor limite de reflejo - Color sensor
#define REFLECTION_LIMIT            30

// Velocidad usando comandos de movimiento relativo y absoluto
#define STEP_ROTATION_SPEED         40
#define STEP_ELEVATION_SPEED        20
#define STEP_CLAW_SPEED             40

// Estado de motor sobrecargado (RUNNING + STALLED)
#define MOTOR_LIMIT                 9

// Posiciones limite no comprobables mediante sensores
#define TOP_BOTTOM_POS              200
#define TOP_LEFT_POS                -400

// Tiempo de espera para cortar la potencia a la garra en el cierre
#define CLAW_CLOSE_TIME             500000 // usec

// LCD
#define X_TITLE                     20
#define Y_TITLE                     10
#define TITLE                       "LEGO - ROBOTIC ARM"
#define X_CIRCLE                    EV3_X_LCD / 2
#define Y_CIRCLE                    EV3_Y_LCD / 2
#define RADIUS                      35
#define COLOR_CIRCLE                1
#define X_TIME                      60
#define Y_TIME                      EV3_Y_LCD - 20

// Periodos (nsec)
#define BUTTON_PERIOD               180000000
#define COLOR_PERIOD                200000000
#define TOUCH_PERIOD                200000000
#define MOTOR_PERIOD                90000000 // Rotation, elevation & claw
#define LED_PERIOD                  40000000
#define REPORTER_PERIOD             500000000

// Stop modes
typedef enum stop_mode_enum {COAST, BRAKE, HOLD} stop_mode;
static char *STOP_MODE_STRING[] = {"coast", "brake", "hold"};

// Commands
typedef enum commands_enum {RUN_FOREVER, RUN_ABS_POS, RUN_REL_POS, RUN_TIMED, RUN_DIRECT, STOP, RESET} commands;
static char *COMMANDS_STRING[] = {"run-forever", "run-to-abs-pos", "run-to-rel-pos", "run-timed", "run-direct",
                                  "stop", "reset"};

// Rotation actions
typedef enum {ROTATE_RIGHT, ROTATE_LEFT, ROTATE_STOP} actions_rotation;

// Elevation actions
typedef enum actions_elevation_enum{RISE, LOWER, ELEVATE_STOP} actions_elevation;

// Claw actions
typedef enum actions_claw_enum {ACTIVE, INACTIVE} actions_claw;

// Color sensor commands
typedef enum color_command_enum
{
    COL_REFLECT, COL_AMBIENT, COL_COLOR
} color_command;

// Nuevas instrucciones para los motores
struct new_motors_status {
	pthread_mutex_t mutex;
	actions_rotation rotation;
	actions_elevation elevation;
	actions_claw claw;
} new_motors_status;

// Parametros para inicializar el motor de rotacion
typedef struct rotation_init_params {
	ev3_motor_ptr rotation_motor;
	ev3_sensor_ptr touch_sensor;
	struct timespec period;
} rotation_init_params_t;

// Parametros para inicializar el motor de elevacion
typedef struct elevation_init_params {
	ev3_motor_ptr elevation_motor;
	ev3_sensor_ptr color_sensor;
	struct timespec period;
} elevation_init_params_t;

// Parametros para inicializar el motor de la garra
typedef struct claw_init_params {
	ev3_motor_ptr claw_motor;
	struct timespec period;
} claw_init_params_t;

// Flag - color sensor with mutex
struct top_limit {
	bool top_limit_reached;
	pthread_mutex_t top_mutex;
} top_limit;

// Flag - touch sensor with mutex
struct clockwise_limit {
	bool clockwise_limit_reached;
	pthread_mutex_t clockwise_mutex;
} clockwise_limit;

// Flag - back button with mutex
struct close_condition {
	bool close;
	pthread_mutex_t close_mutex;
} close_condition;

// Flag - motors running to stable position with mutex
struct correction {
	bool correction_in_progress;
	pthread_mutex_t correction_mutex;
} correction;

// Flag - claw being used -> reporter
struct claw_used {
	bool status;
	pthread_mutex_t claw_used_mutex;
} claw_used;

/*
 * FUNCIONES DE INICIALIZACION
 */

/**
 * @brief Inicializa el motor de rotacion. Para ello, rota hasta alcanzar el fin de carrera
 *        (touch sensor). Desde ahi, rota en sentido contrario un numero de posiciones determinado
 *        para fijar la posición inicial.
 *
 * @param rotation_init_params_t Estructura con el motor de rotacion, el fin de carrera y los
 *                               periodos de muestreo.
 */
void* rotation_motor_initializer (void *params);

/**
 * @brief Inicializa el motor de elevacion. Para ello, eleva hasta alcanzar el valor limite
 *        de luz reflejada y detectada por el sensor de color. Desde ahi, baja un numero de
 *        posiciones determinado para fijar la posicion inicial.
 *
 * @param elevation_init_params_t Estructura con el motor de elevacion, el sensor de color y los
 *                                periodos de muestreo.
 */
void* elevation_motor_initializer (void *params);

/**
 * @brief Inicializa el motor de la garra. Para ello, cierra el motor por completo y vuelve
 *        a abrirlo hasta una posicion inicial un numero de posiciones determinado.
 *
 * @param claw_init_params_t Estructura con el motor de la garra y el periodo de muestreo.
 */
void* claw_motor_initializer (void *params);

/*
 * FUNCIONES PRINCIPALES
 */

/**
 * @brief Controla el motor de rotacion, atendiendo las ordenes recibidas desde la botonera
 *        y teniendo en cuenta los limites (posicion fija + fin de carrera). Si se alcanza
 *        un limite, se rota a la posicion inicial.
 *
 * @param ev3_motor_ptr Motor de rotacion.
 */
void* rotation_motor_controller (void *param);

/**
 * @brief Controla el motor de elevacion, atendiendo las ordenes recibidas desde la botonera
 *        y teniendo en cuenta los limites (posicion fija + sensor de color). Si se alcanza
 *        un limite, se mueve a la posicion inicial.
 *
 * @param ev3_motor_ptr Motor de elevacion.
 */
void* elevation_motor_controller (void *param);

/**
 * @brief Controla el motor de la garra, atendiendo las ordenes recibidas desde la botonera.
 *        El cierre de la garra se adapta al tamaño del objeto agarrado cortando la potencia
 *        despues de un tiempo concreto.
 *
 * @param ev3_motor_ptr Motor de la garra.
 */
void* claw_motor_controller (void *param);

/**
 * @brief Controla la botonera del brick. Mediante una estructura compartida, puede indicar
 *        las acciones solicitadas por el usuario a los motores. Se permiten pulsaciones
 *        simultaneas para movimientos diagonales.
 */
void* buttons_controller (void *params);

/**
 * @brief Controla el sensor de color. Activa una flag cuando se detecta un reflejo superior
 *        a REFLECTION_LIMIT, lo cual significa que el brazo ha alcanzado el limite de altura.
 *
 * @param ev3_sensor_ptr Sensor de color.
 */
void* color_sensor_controller (void *param);

/**
 * @brief Controla el fin de carrera o sensor de pulsacion. Activa una flag cuando se detecta
 *        la pulsacion, lo cual significa que el brazo ha alcanzado el limite de giro en sentido
 *        horario.
 *
 * @param ev3_sensor_ptr Sensor de pulsacion o fin de carrera.
 */
void* touch_sensor_controller (void *param);

/**
 * @brief Controla los leds del brick. Estos se establecen en color verde durante un funcionamiento
 *        normal y en color rojo cuando uno de los motores esta retornando a la posicion inicial
 *        segura por sobrepasar un limite.
 */
void* leds_controller(void *params);

/**
 * @brief Reportero sencillo de informacion. Imprime por pantalla el titulo del programa, una
 *        circunferencia (garra abierta) o un circulo (garra cerrada) y la hora con una precision
 *        de segundos.
 */
void* reporter(void *params);

/*
 * FUNCIONES AUXILIARES
 */

/**
 * @brief Comprueba si se ha activado el flag que señala que se ha alcanzado el limite superior.
 *
 * @return true si se ha alcanzado.
 *         false en caso contrario.
 */
bool is_top_limit_reached();

/**
 * @brief Comprueba si se ha activado el flag que señala que se ha alcanzado el limite de giro
 *        horario.
 *
 * @return true si se ha alcanzado.
 *         false en caso contrario.
 */
bool is_clockwise_limit_reached();

/**
 * @brief Comprueba si se ha activado el flag que señala que se ha pulsado el boton de retroceso
 *        (finalizacion).
 *
 * @return true si se ha alcanzado.
 *         false en caso contrario.
 */
bool is_close_pressed();

/*
 * MAIN
 */

int main(void) {
	/*
	 * CARGA MOTORES Y SENSORES.
	 */

	/* Motores */
	ev3_motor_ptr motors = NULL;

	motors  = ev3_load_motors();
	if (motors == NULL) {
		printf("Error on ev3_load_motors\n");
		return EXIT_FAILURE;
	}

	/* Motor de rotacion */
	ev3_motor_ptr rotation_motor = ev3_search_motor_by_port(motors, LARGE_ROTATION_MOTOR_PORT);
	if (rotation_motor == NULL) {
		printf("Error on ev3_search_motor_by_port with rotation motor.\n");
	    return EXIT_FAILURE;
	}

	ev3_reset_motor(rotation_motor);
	rotation_motor = ev3_open_motor(rotation_motor);
	if (rotation_motor == NULL) {
		printf ("Error on ev3_open_sensor with rotation motor.\n");
		return EXIT_FAILURE;
	}

	/* Motor de elevacion */
	ev3_motor_ptr elevation_motor = ev3_search_motor_by_port(motors, LARGE_ELEVATION_MOTOR_PORT);
	if (rotation_motor == NULL) {
		printf ("Error on ev3_search_motor_by_port with elevation motor.\n");
		return EXIT_FAILURE;
	}

	ev3_reset_motor(elevation_motor);
	elevation_motor = ev3_open_motor(elevation_motor);
	if (elevation_motor == NULL) {
		printf("Error on ev3_open_sensor with elevation motor.\n");
		return EXIT_FAILURE;
	}

	/* Motor de garra */
	ev3_motor_ptr claw_motor = ev3_search_motor_by_port(motors, MEDIUM_CLAW_MOTOR_PORT);
	if (rotation_motor == NULL) {
		printf("Error on ev3_search_motor_by_port with claw motor.\n");
		return EXIT_FAILURE;
	}

	ev3_reset_motor(claw_motor);
	claw_motor = ev3_open_motor(claw_motor);
	if (claw_motor == NULL) {
		printf("Error on ev3_open_sensor with claw motor.\n");
		return EXIT_FAILURE;
	}

	/* Sensores */
	ev3_sensor_ptr sensors = ev3_load_sensors();
	if (sensors == NULL) {
		printf("Error on ev3_load_sensors\n");
		return EXIT_FAILURE;
	}

	// Fin de carrera
	ev3_sensor_ptr touch_sensor = ev3_search_sensor_by_port(sensors, TOUCH_SENSOR_PORT);
	if (touch_sensor == NULL) {
		printf("Error with touch sensor on ev3_search_sensor_by_port.\n");
	    return EXIT_FAILURE;
	}

	touch_sensor = ev3_open_sensor (touch_sensor);
	if (touch_sensor == NULL) {
	    printf ("Error on ev3_open_sensor with touch sensor.\n");
	    return EXIT_FAILURE;
	}

	// Sensor de color
	ev3_sensor_ptr color_sensor = ev3_search_sensor_by_port(sensors, COLOR_SENSOR_PORT);
	if (color_sensor == NULL) {
		printf("Error with color sensor on ev3_search_sensor_by_port.\n");
		return EXIT_FAILURE;
	}

	color_sensor = ev3_open_sensor(color_sensor);
	if (color_sensor == NULL) {
		printf("Error on ev3_open_sensor with color sensor.\n");
		return EXIT_FAILURE;
	}
	ev3_mode_sensor(color_sensor, COL_REFLECT);

	// Botonera
	ev3_init_button();

	// Leds
	ev3_init_led();

	// LCD
	ev3_init_lcd();

	/*
	 * INICIALIZA ROTACION, ELEVACION Y GARRA
	 */

	// Rotation params
	rotation_init_params_t rotation_init_params;
	rotation_init_params.rotation_motor = rotation_motor;
	rotation_init_params.touch_sensor = touch_sensor;
	rotation_init_params.period.tv_sec =  0;
	rotation_init_params.period.tv_nsec = MOTOR_PERIOD;

	// Elevation params
	elevation_init_params_t elevation_init_params;
	elevation_init_params.elevation_motor = elevation_motor;
	elevation_init_params.color_sensor = color_sensor;
	elevation_init_params.period.tv_sec = 0;
	elevation_init_params.period.tv_nsec = MOTOR_PERIOD;

	// Claw params
	claw_init_params_t claw_init_params;
	claw_init_params.claw_motor = claw_motor;
	claw_init_params.period.tv_sec = 0;
	claw_init_params.period.tv_nsec = MOTOR_PERIOD;

	// Prepare thread attributes
	pthread_t th_init_rotation, th_init_elevation, th_init_claw;
	pthread_attr_t th_init_rotation_attr, th_init_elevation_attr, th_init_claw_attr;

	CHK(pthread_attr_init(&th_init_rotation_attr));
	CHK(pthread_attr_setinheritsched(&th_init_rotation_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_init_rotation_attr, SCHED_FIFO));
	struct sched_param sch_param_init_rotation;
	sch_param_init_rotation.sched_priority = sched_get_priority_max(SCHED_FIFO) - 10; // Max = 99
	CHK(pthread_attr_setschedparam(&th_init_rotation_attr, &sch_param_init_rotation));
	CHK(pthread_attr_setdetachstate (&th_init_rotation_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_init_elevation_attr));
	CHK(pthread_attr_setinheritsched(&th_init_elevation_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_init_elevation_attr, SCHED_FIFO));
	struct sched_param sch_param_init_elevation;
	sch_param_init_elevation.sched_priority = sched_get_priority_max(SCHED_FIFO) - 5; // Max = 99
	CHK(pthread_attr_setschedparam(&th_init_elevation_attr, &sch_param_init_elevation));
	CHK(pthread_attr_setdetachstate (&th_init_elevation_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_init_claw_attr));
	CHK(pthread_attr_setinheritsched(&th_init_claw_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_init_claw_attr, SCHED_FIFO));
	struct sched_param sch_param_init_claw;
	sch_param_init_claw.sched_priority = sched_get_priority_max(SCHED_FIFO) - 15; // Max = 99
	CHK(pthread_attr_setschedparam(&th_init_claw_attr, &sch_param_init_claw));
	CHK(pthread_attr_setdetachstate (&th_init_claw_attr, PTHREAD_CREATE_JOINABLE));

	// Create threads
	CHK(pthread_create(&th_init_rotation, &th_init_rotation_attr, rotation_motor_initializer,
			&rotation_init_params));
	CHK(pthread_create(&th_init_elevation, &th_init_elevation_attr, elevation_motor_initializer,
			&elevation_init_params));
	CHK(pthread_create(&th_init_claw, &th_init_claw_attr, claw_motor_initializer,
			&claw_init_params));

	// Espera la finalizacion de todos
	CHK(pthread_join(th_init_rotation, NULL));
	CHK(pthread_join(th_init_elevation, NULL));
	CHK(pthread_join(th_init_claw, NULL));

	// Destruye atributos
	CHK(pthread_attr_destroy(&th_init_rotation_attr));
	CHK(pthread_attr_destroy(&th_init_elevation_attr));
	CHK(pthread_attr_destroy(&th_init_claw_attr));

	// START MAIN PROGRAM

	// Prepare thread attributes
	pthread_t th_rotation, th_elevation, th_claw, th_buttons, th_color_sensor, th_touch_sensor, th_leds, th_reporter;
	pthread_attr_t th_rotation_attr, th_elevation_attr, th_claw_attr, th_buttons_attr, th_color_sensor_attr,
		th_touch_sensor_attr, th_leds_attr, th_reporter_attr;

	CHK(pthread_attr_init(&th_buttons_attr));
	CHK(pthread_attr_setinheritsched(&th_buttons_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_buttons_attr, SCHED_FIFO));
	struct sched_param sch_param_buttons;
	sch_param_buttons.sched_priority = sched_get_priority_max(SCHED_FIFO) - 5; // Max = 99
	CHK(pthread_attr_setschedparam(&th_buttons_attr, &sch_param_buttons));
	CHK(pthread_attr_setdetachstate (&th_buttons_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_color_sensor_attr));
	CHK(pthread_attr_setinheritsched(&th_color_sensor_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_color_sensor_attr, SCHED_FIFO));
	struct sched_param sch_param_color_sensor;
	sch_param_color_sensor.sched_priority = sched_get_priority_max(SCHED_FIFO) - 10; // Max = 99
	CHK(pthread_attr_setschedparam(&th_color_sensor_attr, &sch_param_color_sensor));
	CHK(pthread_attr_setdetachstate (&th_color_sensor_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_touch_sensor_attr));
	CHK(pthread_attr_setinheritsched(&th_touch_sensor_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_touch_sensor_attr, SCHED_FIFO));
	struct sched_param sch_param_touch_sensor;
	sch_param_touch_sensor.sched_priority = sched_get_priority_max(SCHED_FIFO) - 15; // Max = 99
	CHK(pthread_attr_setschedparam(&th_touch_sensor_attr, &sch_param_touch_sensor));
	CHK(pthread_attr_setdetachstate (&th_touch_sensor_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_rotation_attr));
	CHK(pthread_attr_setinheritsched(&th_rotation_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_rotation_attr, SCHED_FIFO));
	struct sched_param sch_param_rotation;
	sch_param_rotation.sched_priority = sched_get_priority_max(SCHED_FIFO) - 20; // Max = 99
	CHK(pthread_attr_setschedparam(&th_rotation_attr, &sch_param_rotation));
	CHK(pthread_attr_setdetachstate (&th_rotation_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_elevation_attr));
	CHK(pthread_attr_setinheritsched(&th_elevation_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_elevation_attr, SCHED_FIFO));
	struct sched_param sch_param_elevation;
	sch_param_elevation.sched_priority = sched_get_priority_max(SCHED_FIFO) - 20; // Max = 99
	CHK(pthread_attr_setschedparam(&th_elevation_attr, &sch_param_elevation));
	CHK(pthread_attr_setdetachstate (&th_elevation_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_claw_attr));
	CHK(pthread_attr_setinheritsched(&th_claw_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_claw_attr, SCHED_FIFO));
	struct sched_param sch_param_claw;
	sch_param_claw.sched_priority = sched_get_priority_max(SCHED_FIFO) - 25; // Max = 99
	CHK(pthread_attr_setschedparam(&th_claw_attr, &sch_param_claw));
	CHK(pthread_attr_setdetachstate (&th_claw_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_leds_attr));
	CHK(pthread_attr_setinheritsched(&th_leds_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_leds_attr, SCHED_FIFO));
	struct sched_param sch_param_leds;
	sch_param_leds.sched_priority = sched_get_priority_max(SCHED_FIFO) - 30; // Max = 99
	CHK(pthread_attr_setschedparam(&th_leds_attr, &sch_param_leds));
	CHK(pthread_attr_setdetachstate (&th_leds_attr, PTHREAD_CREATE_JOINABLE));

	CHK(pthread_attr_init(&th_reporter_attr));
	CHK(pthread_attr_setinheritsched(&th_reporter_attr, PTHREAD_EXPLICIT_SCHED));
	CHK(pthread_attr_setschedpolicy(&th_reporter_attr, SCHED_FIFO));
	struct sched_param sch_param_reporter;
	sch_param_reporter.sched_priority = sched_get_priority_max(SCHED_FIFO) - 35; // Max = 99
	CHK(pthread_attr_setschedparam(&th_reporter_attr, &sch_param_reporter));
	CHK(pthread_attr_setdetachstate (&th_reporter_attr, PTHREAD_CREATE_JOINABLE));

	// Inicializa mutex
	pthread_mutexattr_t top_attr, clock_attr, close_attr, motor_attr, correction_attr, claw_attr;

	CHK(pthread_mutexattr_init(&top_attr));
	CHK(pthread_mutexattr_setprotocol(&top_attr, PTHREAD_PRIO_NONE))
	CHK(pthread_mutex_init(&top_limit.top_mutex, &top_attr));

	CHK(pthread_mutexattr_init(&clock_attr));
	CHK(pthread_mutexattr_setprotocol(&clock_attr, PTHREAD_PRIO_NONE))
	CHK(pthread_mutex_init(&clockwise_limit.clockwise_mutex, &clock_attr));

	CHK(pthread_mutexattr_init(&close_attr));
	CHK(pthread_mutexattr_setprotocol(&close_attr, PTHREAD_PRIO_NONE))
	CHK(pthread_mutex_init(&close_condition.close_mutex, &close_attr));

	CHK(pthread_mutexattr_init(&motor_attr));
	CHK(pthread_mutexattr_setprotocol(&motor_attr, PTHREAD_PRIO_NONE))
	CHK(pthread_mutex_init(&new_motors_status.mutex, &motor_attr));

	CHK(pthread_mutexattr_init(&correction_attr));
	CHK(pthread_mutexattr_setprotocol(&correction_attr, PTHREAD_PRIO_NONE))
	CHK(pthread_mutex_init(&correction.correction_mutex, &correction_attr));

	CHK(pthread_mutexattr_init(&claw_attr));
	CHK(pthread_mutexattr_setprotocol(&claw_attr, PTHREAD_PRIO_NONE))
	CHK(pthread_mutex_init(&claw_used.claw_used_mutex, &claw_attr));

	// Inicializa algunas variables globales
	new_motors_status.claw = INACTIVE;
	new_motors_status.elevation = ELEVATE_STOP;
	new_motors_status.rotation = ROTATE_STOP;
	claw_used.status = false;

	// Create threads
	CHK(pthread_create(&th_buttons, &th_buttons_attr, buttons_controller, (void *) NULL));
	CHK(pthread_create(&th_color_sensor, &th_color_sensor_attr, color_sensor_controller,
			(void *) &color_sensor));
	CHK(pthread_create(&th_touch_sensor, &th_touch_sensor_attr, touch_sensor_controller,
			(void *) &touch_sensor));
	CHK(pthread_create(&th_rotation, &th_rotation_attr, rotation_motor_controller,
			(void *) &rotation_motor));
	CHK(pthread_create(&th_elevation, &th_elevation_attr, elevation_motor_controller,
			(void *) &elevation_motor));
	CHK(pthread_create(&th_claw, &th_claw_attr, claw_motor_controller,
			(void *) &claw_motor));
	CHK(pthread_create(&th_leds, &th_leds_attr, leds_controller, (void *) NULL));
	CHK(pthread_create(&th_reporter, &th_reporter_attr, reporter, (void *) NULL));

	// Finalizacion ordenada
	CHK(pthread_join(th_buttons, NULL));
	CHK(pthread_join(th_color_sensor, NULL));
	CHK(pthread_join(th_touch_sensor, NULL));
	CHK(pthread_join(th_rotation, NULL));
	CHK(pthread_join(th_elevation, NULL));
	CHK(pthread_join(th_claw, NULL));
	CHK(pthread_join(th_leds, NULL));
	CHK(pthread_join(th_reporter, NULL));

	// Destruye atributos y mutex
	CHK(pthread_attr_destroy(&th_buttons_attr));
	CHK(pthread_attr_destroy(&th_color_sensor_attr));
	CHK(pthread_attr_destroy(&th_touch_sensor_attr));
	CHK(pthread_attr_destroy(&th_rotation_attr));
	CHK(pthread_attr_destroy(&th_elevation_attr));
	CHK(pthread_attr_destroy(&th_claw_attr));
	CHK(pthread_attr_destroy(&th_leds_attr));
	CHK(pthread_attr_destroy(&th_reporter_attr));

	CHK(pthread_mutex_destroy(&top_limit.top_mutex));
	CHK(pthread_mutex_destroy(&clockwise_limit.clockwise_mutex));
	CHK(pthread_mutex_destroy(&close_condition.close_mutex));
	CHK(pthread_mutex_destroy(&new_motors_status.mutex));
	CHK(pthread_mutex_destroy(&correction.correction_mutex));
	CHK(pthread_mutex_destroy(&claw_used.claw_used_mutex));

	CHK(pthread_mutexattr_destroy(&top_attr));
	CHK(pthread_mutexattr_destroy(&clock_attr));
	CHK(pthread_mutexattr_destroy(&close_attr));
	CHK(pthread_mutexattr_destroy(&motor_attr));
	CHK(pthread_mutexattr_destroy(&correction_attr));
	CHK(pthread_mutexattr_destroy(&claw_attr));

	// Move to initial position
	ev3_set_position_sp (rotation_motor, 0);
	ev3_command_motor_by_name (rotation_motor, COMMANDS_STRING[RUN_ABS_POS]);
	usleep (SUSPENSION_TIME);
	while ((ev3_motor_state (rotation_motor) & MOTOR_RUNNING)) {
		usleep(CHECK_STATE_TIME);
	}

	ev3_set_position_sp (elevation_motor, 0);
	ev3_command_motor_by_name (elevation_motor, COMMANDS_STRING[RUN_ABS_POS]);
	usleep (SUSPENSION_TIME);
	while ((ev3_motor_state (elevation_motor) & MOTOR_RUNNING)) {
		usleep(CHECK_STATE_TIME);
	}

	ev3_set_position_sp (claw_motor, 0);
	ev3_command_motor_by_name (claw_motor, COMMANDS_STRING[RUN_ABS_POS]);
	usleep (SUSPENSION_TIME);
	while ((ev3_motor_state (claw_motor) & MOTOR_RUNNING)) {
		usleep(CHECK_STATE_TIME);
	}

	// Finaliza
	ev3_reset_motor(rotation_motor);
	ev3_reset_motor(elevation_motor);
	ev3_reset_motor(claw_motor);
	ev3_delete_motors(motors);
	ev3_delete_sensors(sensors);
	ev3_close_sensor(color_sensor);
	ev3_close_sensor(touch_sensor);
	ev3_quit_button();
	ev3_quit_led();
	ev3_clear_lcd();
	ev3_quit_lcd();

	return EXIT_SUCCESS;
}

bool is_close_pressed() {
	bool close_pressed;
	pthread_mutex_lock(&close_condition.close_mutex);
	close_pressed = close_condition.close;
	pthread_mutex_unlock(&close_condition.close_mutex);
	return close_pressed;
}

bool is_clockwise_limit_reached() {
	pthread_mutex_lock(&clockwise_limit.clockwise_mutex);
	bool limit_reached = clockwise_limit.clockwise_limit_reached;
	pthread_mutex_unlock(&clockwise_limit.clockwise_mutex);
	return limit_reached;
}

bool is_top_limit_reached() {
	bool limit_reached;
	pthread_mutex_lock(&top_limit.top_mutex);
	limit_reached = top_limit.top_limit_reached;
	pthread_mutex_unlock(&top_limit.top_mutex);
	return limit_reached;
}


void* rotation_motor_initializer(void *params) {
	rotation_init_params_t *rot_params = (rotation_init_params_t *) params;
	struct timespec next_time;
	clock_gettime(CLOCK_MONOTONIC, &next_time);

	ev3_stop_action_motor_by_name(rot_params->rotation_motor, STOP_MODE_STRING[HOLD]);
	ev3_set_duty_cycle_sp(rot_params->rotation_motor, ROTATION_POWER);
	ev3_command_motor_by_name(rot_params->rotation_motor, COMMANDS_STRING[RUN_DIRECT]);

	int touch_data;

	// Rota hasta alcanzar sensor
	do {
		ev3_update_sensor_val(rot_params->touch_sensor);
		touch_data = rot_params->touch_sensor->val_data[0].s32;

		incr_timespec(&next_time, &rot_params->period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	} while (touch_data == TOUCH_SENSOR_INACTIVE);

	// Rotar 90º (aprox.) counterclockwise
	ev3_set_speed_sp(rot_params->rotation_motor, (STEP_ROTATION_SPEED *
			rot_params->rotation_motor->max_speed) / 100);
	ev3_set_position_sp(rot_params->rotation_motor, ROTATION_INIT_UNITS);
	ev3_command_motor_by_name(rot_params->rotation_motor, COMMANDS_STRING[RUN_REL_POS]);
	usleep(SUSPENSION_TIME);

	clock_gettime(CLOCK_MONOTONIC, &next_time);
	do {
		incr_timespec(&next_time, &rot_params->period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	} while ((ev3_motor_state (rot_params->rotation_motor) & MOTOR_RUNNING));

	ev3_set_duty_cycle_sp(rot_params->rotation_motor, 0);
	ev3_command_motor_by_name(rot_params->rotation_motor, COMMANDS_STRING[RUN_DIRECT]);
	ev3_set_position(rot_params->rotation_motor, 0);

	pthread_exit(NULL);
}

void* elevation_motor_initializer(void *params) {
	elevation_init_params_t *elev_params = (elevation_init_params_t *) params;
	struct timespec next_time;
	clock_gettime(CLOCK_MONOTONIC, &next_time);

	ev3_stop_action_motor_by_name (elev_params->elevation_motor, STOP_MODE_STRING[HOLD]);
	ev3_set_duty_cycle_sp (elev_params->elevation_motor, ELEVATION_UP_POWER);
	ev3_command_motor_by_name (elev_params->elevation_motor, COMMANDS_STRING[RUN_DIRECT]);

	int reflection_data;

	// Elevar hasta que se pasa el limite de REFLECTION_LIMIT
	do {
		ev3_update_sensor_val(elev_params->color_sensor);
		reflection_data = elev_params->color_sensor->val_data[0].s32;
		incr_timespec(&next_time, &elev_params->period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	} while (reflection_data < REFLECTION_LIMIT);

	// Lower 45º (aprox.)
	ev3_set_speed_sp(elev_params->elevation_motor, (STEP_ELEVATION_SPEED *
			elev_params->elevation_motor->max_speed) / 100);
	ev3_set_position_sp(elev_params->elevation_motor, ELEVATION_INIT_UNITS);
	ev3_command_motor_by_name(elev_params->elevation_motor, COMMANDS_STRING[RUN_REL_POS]);
	usleep(SUSPENSION_TIME);
	clock_gettime(CLOCK_MONOTONIC, &next_time);

	do {
		incr_timespec(&next_time, &elev_params->period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	} while ((ev3_motor_state (elev_params->elevation_motor) & MOTOR_RUNNING));

	ev3_set_duty_cycle_sp(elev_params->elevation_motor, 0);
	ev3_command_motor_by_name(elev_params->elevation_motor, COMMANDS_STRING[RUN_DIRECT]);
	ev3_set_position(elev_params->elevation_motor, 0);
	pthread_exit(NULL);
}

void* claw_motor_initializer(void* params) {
	claw_init_params_t *claw_params = (claw_init_params_t *) params;
	struct timespec next_time;
	clock_gettime(CLOCK_MONOTONIC, &next_time);

	ev3_stop_action_motor_by_name(claw_params->claw_motor, STOP_MODE_STRING[HOLD]);
	ev3_set_duty_cycle_sp(claw_params->claw_motor, -CLAW_POWER);
	ev3_command_motor_by_name(claw_params->claw_motor, COMMANDS_STRING[RUN_DIRECT]);

	int claw_status;

	do {
		claw_status = ev3_motor_state(claw_params->claw_motor);
		incr_timespec(&next_time, &claw_params->period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	} while (claw_status != MOTOR_LIMIT);

	ev3_set_speed_sp(claw_params->claw_motor, (STEP_CLAW_SPEED * claw_params->claw_motor->max_speed) / 100);
	ev3_set_position_sp(claw_params->claw_motor, CLAW_INIT_UNITS);
	ev3_command_motor_by_name(claw_params->claw_motor, COMMANDS_STRING[RUN_REL_POS]);
	usleep(SUSPENSION_TIME);

	do {
		incr_timespec(&next_time, &claw_params->period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	} while ((ev3_motor_state (claw_params->claw_motor) & MOTOR_RUNNING));

	ev3_set_duty_cycle_sp(claw_params->claw_motor, 0);
	ev3_command_motor_by_name(claw_params->claw_motor, COMMANDS_STRING[RUN_DIRECT]);
	ev3_set_position(claw_params->claw_motor, 0);

	pthread_exit(NULL);
}

void* buttons_controller(void *params) {
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = BUTTON_PERIOD;

	while(!is_close_pressed()) {
		pthread_mutex_lock(&new_motors_status.mutex);
		// Rotation buttons
		if (ev3_button_pressed(BUTTON_LEFT)) { // If left pressed
			if (ev3_button_pressed(BUTTON_RIGHT)) { // And right at the same time
				new_motors_status.rotation = ROTATE_STOP;
			} else { // Only left
				new_motors_status.rotation = ROTATE_LEFT;
			}
		} else if (ev3_button_pressed(BUTTON_RIGHT)) { // Only right
				new_motors_status.rotation = ROTATE_RIGHT;
		} else { // No button pressed
			new_motors_status.rotation = ROTATE_STOP;
		}

		// Elevation buttons
		if (ev3_button_pressed(BUTTON_UP)) {
			if (ev3_button_pressed(BUTTON_DOWN)) {
				new_motors_status.elevation = ELEVATE_STOP;
			} else {
				new_motors_status.elevation = RISE;
			}
		} else if (ev3_button_pressed(BUTTON_DOWN)) {
			new_motors_status.elevation = LOWER;
		} else {
			new_motors_status.elevation = ELEVATE_STOP;
		}

		// Claw button
		if (ev3_button_pressed(BUTTON_CENTER)) {
			new_motors_status.claw = ACTIVE;
		} else {
			new_motors_status.claw = INACTIVE;
		}

		pthread_mutex_unlock(&new_motors_status.mutex);

		// Cancel button
		pthread_mutex_lock(&close_condition.close_mutex);
		if (ev3_button_pressed(BUTTON_BACK)) {
			close_condition.close = true;
		}
		pthread_mutex_unlock(&close_condition.close_mutex);
		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}
	pthread_exit(NULL);
}

void* color_sensor_controller (void *param) {
	ev3_sensor_ptr color_sensor = *((ev3_sensor_ptr *) param);
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = COLOR_PERIOD;
	int color_data;

	while (!is_close_pressed()) {
		ev3_update_sensor_val(color_sensor);
		color_data = color_sensor->val_data[0].s32;
		if (color_data >= REFLECTION_LIMIT) {
			pthread_mutex_lock(&top_limit.top_mutex);
			top_limit.top_limit_reached = true;
			pthread_mutex_unlock(&top_limit.top_mutex);
		}
		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}

	pthread_exit(NULL);
}

void* touch_sensor_controller (void *param) {
	ev3_sensor_ptr touch_sensor = *((ev3_sensor_ptr *) param);
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = TOUCH_PERIOD;

	int touch_data;

	while (!is_close_pressed()) {
		ev3_update_sensor_val(touch_sensor);
		touch_data = touch_sensor->val_data[0].s32;
		if (touch_data == TOUCH_SENSOR_ACTIVE) {
			pthread_mutex_lock(&clockwise_limit.clockwise_mutex);
			clockwise_limit.clockwise_limit_reached = true;
			pthread_mutex_unlock(&clockwise_limit.clockwise_mutex);
		}
		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}

	pthread_exit(NULL);
}

void* rotation_motor_controller (void *param) {
	ev3_motor_ptr rotation_motor = *((ev3_motor_ptr *) param);
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = MOTOR_PERIOD;

	actions_rotation rotation_actual = ROTATE_STOP;
	actions_rotation rotation_next = ROTATE_STOP;

	while(!is_close_pressed()) {
		pthread_mutex_lock(&new_motors_status.mutex);
		rotation_next = new_motors_status.rotation;
		pthread_mutex_unlock(&new_motors_status.mutex);

		if (is_clockwise_limit_reached()) {
			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = true;
			pthread_mutex_unlock(&correction.correction_mutex);
			ev3_set_position_sp(rotation_motor, ROTATION_INIT_UNITS);
			ev3_command_motor_by_name(rotation_motor, COMMANDS_STRING[RUN_REL_POS]);
			usleep (SUSPENSION_TIME);

			while ((ev3_motor_state(rotation_motor) & MOTOR_RUNNING)) {
				usleep(CHECK_STATE_TIME);
			}

			pthread_mutex_lock(&clockwise_limit.clockwise_mutex);
			clockwise_limit.clockwise_limit_reached = false;
			pthread_mutex_unlock(&clockwise_limit.clockwise_mutex);

			ev3_set_duty_cycle_sp(rotation_motor, 0);
			ev3_command_motor_by_name(rotation_motor, COMMANDS_STRING[RUN_DIRECT]);
			rotation_actual = ROTATE_STOP;

			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = false;
			pthread_mutex_unlock(&correction.correction_mutex);

		} else if (ev3_get_position(rotation_motor) < TOP_LEFT_POS) {
			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = true;
			pthread_mutex_unlock(&correction.correction_mutex);
			ev3_set_position_sp(rotation_motor, 0);
			ev3_command_motor_by_name(rotation_motor, COMMANDS_STRING[RUN_ABS_POS]);
			usleep(SUSPENSION_TIME);

			while ((ev3_motor_state (rotation_motor) & MOTOR_RUNNING)) {
				usleep(CHECK_STATE_TIME);
			}

			ev3_set_duty_cycle_sp(rotation_motor, 0);
			ev3_command_motor_by_name(rotation_motor, COMMANDS_STRING[RUN_DIRECT]);
			rotation_actual = ROTATE_STOP;

			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = false;
			pthread_mutex_unlock(&correction.correction_mutex);

		} else if (rotation_actual != rotation_next) {
			switch(rotation_next) {
				case ROTATE_RIGHT:
					ev3_set_duty_cycle_sp (rotation_motor, ROTATION_POWER);
					break;
				case ROTATE_LEFT:
					ev3_set_duty_cycle_sp (rotation_motor, -ROTATION_POWER);
					break;
				default:
					ev3_set_duty_cycle_sp(rotation_motor, 0);
					break;
			}
			rotation_actual = rotation_next;
		}
		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}

	pthread_exit(NULL);
}

void* elevation_motor_controller (void *param) {

	ev3_motor_ptr elevation_motor = *((ev3_motor_ptr *) param);
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = MOTOR_PERIOD;

	actions_elevation elevation_actual = ELEVATE_STOP;
	actions_elevation elevation_next = ELEVATE_STOP;

	while(!is_close_pressed()) {
		pthread_mutex_lock(&new_motors_status.mutex);
		elevation_next = new_motors_status.elevation;
		pthread_mutex_unlock(&new_motors_status.mutex);

		if (is_top_limit_reached()) {
			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = true;
			pthread_mutex_unlock(&correction.correction_mutex);
			ev3_set_position_sp(elevation_motor, ELEVATION_INIT_UNITS);
			ev3_command_motor_by_name(elevation_motor, COMMANDS_STRING[RUN_REL_POS]);
			usleep (SUSPENSION_TIME);

			while ((ev3_motor_state (elevation_motor) & MOTOR_RUNNING)) {
				usleep(CHECK_STATE_TIME);
			}

			pthread_mutex_lock(&top_limit.top_mutex);
			top_limit.top_limit_reached = false;
			pthread_mutex_unlock(&top_limit.top_mutex);

			ev3_set_duty_cycle_sp (elevation_motor, 0);
			ev3_command_motor_by_name (elevation_motor, COMMANDS_STRING[RUN_DIRECT]);
			elevation_actual = ELEVATE_STOP;

			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = false;
			pthread_mutex_unlock(&correction.correction_mutex);

		} else if (ev3_get_position(elevation_motor) > TOP_BOTTOM_POS) {
			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = true;
			pthread_mutex_unlock(&correction.correction_mutex);

			ev3_set_position_sp(elevation_motor, 0);
			ev3_command_motor_by_name(elevation_motor, COMMANDS_STRING[RUN_ABS_POS]);

			usleep(SUSPENSION_TIME);
			while ((ev3_motor_state(elevation_motor) & MOTOR_RUNNING)) {
				usleep(CHECK_STATE_TIME);
			}

			ev3_set_duty_cycle_sp(elevation_motor, 0);
			ev3_command_motor_by_name(elevation_motor, COMMANDS_STRING[RUN_DIRECT]);
			elevation_actual = ELEVATE_STOP;
			pthread_mutex_lock(&correction.correction_mutex);
			correction.correction_in_progress = false;
			pthread_mutex_unlock(&correction.correction_mutex);

		} else if (elevation_actual != elevation_next) {
			switch(elevation_next) {
				case RISE:
					ev3_set_duty_cycle_sp (elevation_motor, ELEVATION_UP_POWER);
					break;
				case LOWER:
					ev3_set_duty_cycle_sp (elevation_motor, ELEVATION_DOWN_POWER);
					break;
				default:
					ev3_set_duty_cycle_sp(elevation_motor, 0);
					break;
			}
			elevation_actual = elevation_next;
		}
		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}

	pthread_exit(NULL);
}

void* claw_motor_controller (void *param) {
	ev3_motor_ptr claw_motor = *((ev3_motor_ptr *) param);
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = MOTOR_PERIOD;

	bool claw_open = true;
	actions_claw claw_status = INACTIVE;

	while(!is_close_pressed()) {
		pthread_mutex_lock(&new_motors_status.mutex);
		claw_status = new_motors_status.claw;
		pthread_mutex_unlock(&new_motors_status.mutex);

		if (claw_status == ACTIVE) {
			if (claw_open) {
				ev3_set_duty_cycle_sp (claw_motor, -CLAW_POWER);
				ev3_command_motor_by_name (claw_motor, COMMANDS_STRING[RUN_DIRECT]);
				claw_open = false;

				usleep(CLAW_CLOSE_TIME);
				ev3_set_duty_cycle_sp (claw_motor, 0);
				pthread_mutex_lock(&claw_used.claw_used_mutex);
				claw_used.status = true;
				pthread_mutex_unlock(&claw_used.claw_used_mutex);
			} else {
				ev3_set_position_sp (claw_motor, 0);
				ev3_command_motor_by_name (claw_motor, COMMANDS_STRING[RUN_ABS_POS]);
				usleep (SUSPENSION_TIME);

				while ((ev3_motor_state (claw_motor) & MOTOR_RUNNING)) {
					usleep(CHECK_STATE_TIME);
				}

				ev3_set_duty_cycle_sp (claw_motor, 0);
				ev3_command_motor_by_name (claw_motor, COMMANDS_STRING[RUN_DIRECT]);
				claw_open = true;
				pthread_mutex_lock(&claw_used.claw_used_mutex);
				claw_used.status = false;
				pthread_mutex_unlock(&claw_used.claw_used_mutex);
			}
			pthread_mutex_lock(&new_motors_status.mutex);
			new_motors_status.claw = INACTIVE;
			pthread_mutex_unlock(&new_motors_status.mutex);
		}
		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}
	pthread_exit(NULL);
}

void* leds_controller(void *params) {
	bool previous = false;
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = LED_PERIOD;
	bool actual;

	while(!is_close_pressed()) {
		pthread_mutex_lock(&correction.correction_mutex);
		actual =correction.correction_in_progress;
		pthread_mutex_unlock(&correction.correction_mutex);
		if (actual && !previous) {
			ev3_set_led(LEFT_LED , RED_LED , 255);
			ev3_set_led(RIGHT_LED, RED_LED, 255);
			ev3_set_led(LEFT_LED , GREEN_LED , 0);
			ev3_set_led(RIGHT_LED , GREEN_LED , 0);
			previous = true;
		} else if (!actual && previous) {
			ev3_set_led(LEFT_LED , GREEN_LED , 255);
			ev3_set_led(RIGHT_LED, GREEN_LED, 255);
			ev3_set_led(LEFT_LED , RED_LED , 0);
			ev3_set_led(RIGHT_LED, RED_LED, 0);
			previous = false;
		}
		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}
	pthread_exit(NULL);
}

void* reporter(void *params) {
	struct timespec next_time, period;
	clock_gettime(CLOCK_MONOTONIC, &next_time);
	period.tv_sec = 0;
	period.tv_nsec = REPORTER_PERIOD;
	bool claw_status;
	time_t now;
	struct tm *now_tm;
	char time_str[9];
	int hour;
	int minute;
	int second;

	while(!is_close_pressed()) {
		ev3_clear_lcd();
		pthread_mutex_lock(&claw_used.claw_used_mutex);
		claw_status = claw_used.status;
		pthread_mutex_unlock(&claw_used.claw_used_mutex);

		time(&now);
		now = time(NULL);
		now_tm = localtime(&now);

		hour = now_tm->tm_hour;
		minute = now_tm->tm_min;
		second = now_tm->tm_sec;
		sprintf(time_str, "%02d:%02d:%02d", hour, minute, second);

		ev3_text_lcd_normal(X_TITLE, Y_TITLE, TITLE);
		if(claw_status) {
			ev3_circle_lcd(X_CIRCLE, Y_CIRCLE, RADIUS, COLOR_CIRCLE);
		} else {
			ev3_circle_lcd_out(X_CIRCLE, Y_CIRCLE, RADIUS, COLOR_CIRCLE);
		}
		ev3_text_lcd_normal(X_TIME, Y_TIME, time_str);

		incr_timespec(&next_time, &period);
		CHK(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL));
	}
	pthread_exit(NULL);
}
