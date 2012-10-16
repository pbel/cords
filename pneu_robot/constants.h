
//#ifndef _CONSTANTS_H
//#define _CONSTANTS_H

// Global constants

int		RAND_SEED			= 2;

double		STEP_SIZE			= 0.05;

// Controls the speed vs. accuracy of the simulator:
// A larger step size makes the simulator to run faster,
// but less stably;
// A smaller step size causes the simulator to run slower,
// but more stably.


// Environment-related constants

int		MAX_ENVIRONMENTS		= 4;

int		MAX_OTHER_OBJECTS		= 20;

int		SELECTION_LEVEL_ROBOT		= 0;
int		SELECTION_LEVEL_OBJECT		= 1;
int		SELECTION_LEVEL_ENVIRONMENT	= 2;
int		SELECTION_LEVEL_ENVS		= 3;

// Optimizer-related constants

//int		STARTING_EVALUATION_TIME	= 500;		//*****
//int		STARTING_EVALUATION_TIME	= 1500;
int		STARTING_EVALUATION_TIME	= 25000;
double		MUTATION_PROBABILITY		= 0.05;

int		EVALS_UNTIL_EVAL_TIME_EXTENDED	= 30;
int		EVAL_TIME_EXTENSION_PERIOD	= 10;

double		WORST_FITNESS			= -1000000.0;

//int		MAX_EVALS_BEFORE_SAVING		= 100;		//*****
int		MAX_EVALS_BEFORE_SAVING		= 10;

int		TIME_TO_CHECK_FOR_NEUTRAL_MUTATION = 50;

//	ALPS-related constants

//int		ALPS_NUM_LAYERS			= 60;		//*****
int		ALPS_NUM_LAYERS			= 10;

//int		ALPS_GENOMES_PER_LAYER		= 5;		//*****
int		ALPS_GENOMES_PER_LAYER		= 3;

// Constants related to viewing the simulation

int		MODE_VIEW_SIDE			= 0;
int		MODE_VIEW_TOP			= 1;
int		MODE_VIEW_BACK			= 2;

int		MODE_SIMULATE_DESIGN		= 0;
int		MODE_SIMULATE_EVOLVE		= 1;
int		MODE_SIMULATE_CHAMP		= 2;
int		MODE_SIMULATE_FILE		= 3;

double		MOVE_INCREMENT			= 0.1;
double		ROT_INCREMENT			= M_PI/60;// 3 deg, in radians

// Object constants

int		SHAPE_RECTANGLE			= 0;
int		SHAPE_CYLINDER			= 1;
int		SHAPE_SPHERE			= 2;
int		SHAPE_HINGE			= 3;

double		JOINT_PIN_LENGTH		= 0.3;
double		JOINT_PIN_RADIUS		= 0.02;

int		OBJECT_STATE_INCORPOREAL	= 0;
int		OBJECT_STATE_SOLID		= 1;
int		OBJECT_STATE_PHYSICAL		= 2;

int		OBJECT_FACE_FRONT		= 0;
int		OBJECT_FACE_BACK		= 1;
int		OBJECT_FACE_LEFT		= 2;
int		OBJECT_FACE_RIGHT		= 3;
int		OBJECT_FACE_TOP			= 4;
int		OBJECT_FACE_BOTTOM		= 5;

int		GEOMETRICAL			= 0;
int		PHYSICAL			= 1;


// Light source constants

double		LIGHT_SOURCE_LENGTH		= 0.5;
double		LIGHT_SOURCE_DISTANCE		= 5.0;

double		MAX_LIGHT_SENSOR_DISTANCE	= 40;

// Robot-specific constants

int		MAX_ROBOTS			= 8;
int		MAX_COMPONENTS			= 120;
int		MAX_JOINTS			= 120;

int		ROBOT_STARFISH			= 0;
int		ROBOT_SNAKE			= 1;
int		ROBOT_QUADRUPED			= 2;
int		ROBOT_PNEU_QUAD			= 3;
int		ROBOT_SANDBOX			= 99;

int		GENERIC_JOINT			= 0;
int		ROBOT_BODY_JOINT		= 1;
int		ROBOT_LEG_JOINT			= 2;
int		ROBOT_BODY_TO_LEG_JOINT		= 3;

double		ROBOT_PNEU_QUAD_BODY_LENGTH	= 1.695;
double		ROBOT_PNEU_QUAD_BODY_WIDTH	= 1.43;
double		ROBOT_PNEU_QUAD_BODY_HEIGHT	= 0.13;
double		ROBOT_PNEU_QUAD_CYL_RADIUS	= 0.065;
double		ROBOT_PNEU_QUAD_CYL_LENGTH	= 0.1;
double		ROBOT_PNEU_QUAD_LEG_ANGLE_DEG   = 20;
double		ROBOT_PNEU_QUAD_LEG_HEIGHT	= 0.13;
double		ROBOT_PNEU_QUAD_LEG_LENGTH	= 1.43;
double		ROBOT_PNEU_QUAD_LEG_WIDTH_START	= 0.75;
double		ROBOT_PNEU_QUAD_LEG_WIDTH_END	= 0.475;
int		ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS = 7;
int		ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS = 5;
int		ROBOT_PNEU_QUAD_NUM_LEGS        = 4;

double		ROBOT_PNEU_QUAD_EXT_RANGE	= 0.0;
double		ROBOT_PNEU_QUAD_LEG_FLEX_RANGE	= 90.0;
double		ROBOT_PNEU_QUAD_BODY_FLEX_RANGE	= 60.0;
//double		ROBOT_PNEU_QUAD_MOTOR_STRENGTH	= 1.5;
//double		ROBOT_PNEU_QUAD_MOTOR_STRENGTH	= 7.5; // walk
double		ROBOT_PNEU_QUAD_MOTOR_STRENGTH	= 20; //undulate
//double		ROBOT_PNEU_QUAD_MOTOR_SPEED	= 5.0;
double		ROBOT_PNEU_QUAD_MOTOR_SPEED	= 1.0;

double		ROBOT_STARFISH_BODY_LENGTH	= 1.0;
double		ROBOT_STARFISH_BODY_WIDTH	= 1.0;
double		ROBOT_STARFISH_BODY_HEIGHT	= 0.1;
double		ROBOT_STARFISH_LEG_RADIUS	= 0.1;
double		ROBOT_STARFISH_LEG_LENGTH	= 1.0;

double		ROBOT_STARFISH_JOINT_RANGE	= 10.0;
double		ROBOT_STARFISH_MOTOR_STRENGTH	= 1.5;
double		ROBOT_STARFISH_MOTOR_SPEED	= 5.0;

double		ROBOT_SANDBOX_BOX_LENGTH	= 1.0;
double		ROBOT_SANDBOX_BOX_WIDTH		= 1.0;
double		ROBOT_SANDBOX_BOX_HEIGHT	= 0.1;
double		ROBOT_SANDBOX_CYL_RADIUS	= 0.1;
double		ROBOT_SANDBOX_CYL_LENGTH	= 1.0;

double		ROBOT_SANDBOX_JOINT_RANGE	= 10.0;
double		ROBOT_JOINT_RANGE_DELTA		= 2.0;
double		ROBOT_SANDBOX_MOTOR_STRENGTH	= 1.5;
double		ROBOT_SANDBOX_MOTOR_SPEED	= 5.0;

// Controller-specific constants

int		NODES_PER_MOTOR			= 4;
//int		TIME_STEPS_PER_INTERVAL		= 40;  // for walk test -- use this
int		TIME_STEPS_PER_INTERVAL		= 10;  // for undulation -- use this

//int		NUM_TIME_INTERVALS		= 4;   // for walk
int		NUM_TIME_INTERVALS		= 69;  // for undulation

int		CNTRLR_MOTOR_INIT		= 1; // TBD OK for now
						     // 0: zeros
						     // 1: ones
						     // 2: random
						     // 3: walk (integrated code)
						     // 4: undulate (integrated code)


// Movie-specific constants

//int		TIME_STEPS_PER_FRAME		= 4;
int		TIME_STEPS_PER_FRAME		= 1;

//#endif
