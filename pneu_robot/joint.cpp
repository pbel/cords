#include "stdio.h"
#include "iostream"
#include "fstream"

using namespace std;

#ifndef _JOINT_CPP
#define _JOINT_CPP

#include "joint.h"

#include "robot.h"

#ifdef dDOUBLE
#define dsDrawLine dsDrawLineD
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

// TBD: find a better way to define the robot type
// TBD: find a better way to make values available
extern int	ROBOT_STARFISH;
extern int	ROBOT_SNAKE;
extern int	ROBOT_QUADRUPED;
extern int	ROBOT_PNEU_QUAD;
extern int	ROBOT_SANDBOX;

extern int	GENERIC_JOINT;
extern int	ROBOT_BODY_JOINT;
extern int	ROBOT_LEG_JOINT;
extern int	ROBOT_BODY_TO_LEG_JOINT;

extern double	ROBOT_PNEU_QUAD_EXT_RANGE;
extern double	ROBOT_PNEU_QUAD_FLEX_RANGE;
extern double	ROBOT_PNEU_QUAD_MOTOR_SPEED;
extern double	ROBOT_PNEU_QUAD_MOTOR_STRENGTH;
extern int	ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS;
extern int	ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;

extern double	ROBOT_STARFISH_JOINT_RANGE;
extern double	ROBOT_STARFISH_MOTOR_SPEED;
extern double	ROBOT_STARFISH_MOTOR_STRENGTH;

extern double	JOINT_PIN_LENGTH;
extern double	JOINT_PIN_RADIUS;

JOINT::JOINT(void) {

	containerRobot = NULL;

	obj1Index = -1;
	obj2Index = -1;

	physicalized = false;

	jointType = GENERIC_JOINT;

	active = false;

	markIndex = -1;

	groupID = -1;

	angle = 0;

	hidden = false;
}

JOINT::JOINT(ROBOT *cR, int o1Index, int o2Index,
	     int jType,
	     double posX, double posY, double posZ,
	     double axX,  double axY,  double axZ,
	     double maxF, double maxE) {

	Initialize(o1Index, o2Index, jType,
		   posX, posY, posZ, 
		   axX, axY, axZ, 
		   maxF, maxE);

	proprioceptiveSensor = NULL;

	physicalized = false;

	active = false;

	markIndex = -1;

	groupID = -1;

	angle = 0;

	hidden = true;

	containerRobot = cR;
}

JOINT::JOINT(ROBOT *cR, int o1Index, int o2Index,
	     int jType,
	     double posX, double posY, double posZ,
	     double axX,  double axY,  double axZ,
	     double maxF, double maxE, 
	     int gID) {

	Initialize(o1Index, o2Index, jType,
		   posX, posY, posZ, 
		   axX, axY, axZ, 
		   maxF, maxE);

	proprioceptiveSensor = NULL;

	physicalized = false;

	active = false;

	markIndex = -1;

	groupID = gID;

	angle = 0;

	hidden = true;

	containerRobot = cR;

//	printf("jointType: %d, maxF: %f, maxE: %f\n",jointType,maxFlexion,maxExtension);
}

JOINT::JOINT(ROBOT *cR, JOINT *other) {

	Initialize(other->obj1Index, other->obj2Index, other->jointType,
		   other->x, other->y, other->z,
		   other->axisX, other->axisY, other->axisZ,
		   other->maxFlexion, other->maxExtension);

	Set_Color(2.0, 0.8, 0.0);

	if ( other->proprioceptiveSensor )
		proprioceptiveSensor = new PROP_SENSOR();
	else
		proprioceptiveSensor = NULL;

	physicalized = false;

	active = false;

	markIndex = -1;

	groupID = other->groupID;

	angle = other->angle;

	hidden = false;

	containerRobot = cR;
}

JOINT::JOINT(ROBOT *cR, ifstream *inFile) {

	containerRobot = cR;

	(*inFile) >> obj1Index	>> obj2Index;

	(*inFile) >> color[0] >> color[1] >> color[2];

	(*inFile) >> x 		>>  y 		>> z;

	(*inFile) >> axisX 	>>  axisY 	>> axisZ;

	(*inFile) >> maxFlexion	>> maxExtension;

	(*inFile) >> groupID;

	(*inFile) >> angle;

	(*inFile) >> jointType;

	int containsProprioceptiveSensor;

	(*inFile) >> containsProprioceptiveSensor;

	if ( containsProprioceptiveSensor )

		proprioceptiveSensor = new PROP_SENSOR;
	else
		proprioceptiveSensor = NULL;

	physicalized = false;

	active = false;

	markIndex = -1;

	hidden = true;
}

JOINT::~JOINT(void) {

	if ( physicalized )
		Remove_From_Simulator();

	if ( containerRobot )
		containerRobot = NULL;

	if ( proprioceptiveSensor ) {
		delete proprioceptiveSensor;
		proprioceptiveSensor = NULL;
	}
}

void JOINT::Activate(void) {

	active = true;
}

void JOINT::Deactivate(void) {

	active = false;
}

void JOINT::Disconnect(void) {

	obj1Index = -1;

	obj2Index = -1;
}

void JOINT::Draw(void) {

	if ( hidden )
		return;

	if ( active )
		dsSetColor(color[0], color[1], color[2]);
	else
		dsSetColorAlpha(color[0], color[1], color[2], 0.5);

	Draw_Geometrically();
}

void JOINT::Hide(void) {

	hidden = true;
}

int JOINT::Is_Unconnected(void) {

	return ( obj1Index > containerRobot->numObjects-1
	      || obj2Index > containerRobot->numObjects-1
	      || obj1Index < 0
	      || obj2Index < 0 );
}

void JOINT::Make_Incorporeal(void) {

	// Remove the joint from the physical simulator.

	if ( !physicalized )

		return;

	Remove_From_Simulator();

	Sensors_Reset();
}

void JOINT::Make_Physical(dWorldID world) {

	// Add the joint to the physical simulator.

	// Already added to the simulator.
	if ( physicalized )
		return;

	// If the robot doesn't exist...
	if ( !containerRobot )
		return;

	// if the joint is unconnected ...
	if ( Is_Unconnected() )
		return; 

	physicalized = true;

	joint = dJointCreateHinge(world,0);

	dJointAttach(	joint,
			containerRobot->objects[obj1Index]->body,
			containerRobot->objects[obj2Index]->body);

	dJointSetHingeAnchor(joint,x,y,z);

	dJointSetHingeAxis(joint,axisX,axisY,axisZ);

	dJointSetHingeParam(joint,dParamLoStop,
		Degrees_To_Radians(maxFlexion));

	dJointSetHingeParam(joint,dParamHiStop,
		Degrees_To_Radians(maxExtension));
}

void JOINT::Mark(void) {

	Mark_Joint();
}

void JOINT::Move(double motorNeuronValue) {

	// This joint will apply torque to the two objects
	// it connects, which will cause the robot (of which
	// this joint is a part) to move.

        dJointSetHingeParam(joint,dParamFMax,ROBOT_STARFISH_MOTOR_STRENGTH); // Motor force

	// Scale the motor neuron's value, which is in the range
	// [-1,1], to the joint's range of motion.
	double desiredAngle = Scale(motorNeuronValue,-1,+1,
					-ROBOT_STARFISH_JOINT_RANGE,
					+ROBOT_STARFISH_JOINT_RANGE);

//	printf("%3.3f %3.3f\n",motorNeuronValue,desiredAngle);
//	       desiredAngle = Degrees_To_Radians( desiredAngle );

        double actualAngle =  dJointGetHingeAngle(joint);
        double actualRate = dJointGetHingeAngleRate(joint);

        double ks = ROBOT_STARFISH_MOTOR_SPEED; // Motor speed
        double kd = 0.0;
        double error = 	ks*(desiredAngle - actualAngle) -
			kd*actualRate;

        dJointSetHingeParam(joint,dParamVel,error);
}

void JOINT::Move(int robotType, double motorNeuronValue) {

	// This joint will apply torque to the two objects
	// it connects, which will cause the robot (of which
	// this joint is a part) to move.

	dReal motorStrength = 1.2 * ROBOT_PNEU_QUAD_MOTOR_STRENGTH;

	if ( robotType != ROBOT_PNEU_QUAD ) 

		Move (motorNeuronValue); 

	else {

		if (jointType == ROBOT_BODY_JOINT)
// TBD:  large numbers of components need stronger motors despite mass normalization

//			motorStrength = ROBOT_PNEU_QUAD_MOTOR_STRENGTH /
//				(dReal)(ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS - 1); 
//			motorStrength = ROBOT_PNEU_QUAD_MOTOR_STRENGTH *
//				log10((dReal)(ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS)); 
			motorStrength = ROBOT_PNEU_QUAD_MOTOR_STRENGTH *
				pow(1.012,(dReal)(ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS)); 

		else if (jointType == ROBOT_LEG_JOINT)

//			motorStrength = ROBOT_PNEU_QUAD_MOTOR_STRENGTH /
//				(dReal)(ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS - 1); 
//			motorStrength = ROBOT_PNEU_QUAD_MOTOR_STRENGTH *
//				log10((dReal)(ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS)); 
			motorStrength = ROBOT_PNEU_QUAD_MOTOR_STRENGTH *
				pow(1.012,(dReal)(ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS)); 

		//printf("jointType: %d; motorStrength: %f\n",jointType,motorStrength); 

		dJointSetHingeParam(joint,dParamFMax,motorStrength);// Motor force

		double actualAngle = dJointGetHingeAngle(joint);
		double actualRate  = dJointGetHingeAngleRate(joint);

// TBD: hack for test:
		double desiredAngle = motorNeuronValue; 

// TBD: hacked for test
//		double desiredAngle = ( mValue < -0.00000001 ) ?
//			Degrees_To_Radians(20.0) :
// 			Degrees_To_Radians(0.0);

//		printf("mNV: %3.3f desired angle: %3.3f actual angle: %3.3f\n",
//		       motorNeuronValue,desiredAngle,actualAngle);

		double ks = ROBOT_PNEU_QUAD_MOTOR_SPEED; // Motor speed
		double kd = 0.0;

//		if ( jointType == ROBOT_BODY_JOINT)
//			kd = 0.05 * (dReal)(ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS - 1); 
//		if ( jointType == ROBOT_LEG_JOINT)
//			kd = 0.05 * (dReal)(ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS - 1); 

		double error = 	ks*(desiredAngle - actualAngle) - 
			kd*actualRate;

		dJointSetHingeParam(joint,dParamVel,error);
	}
}

void JOINT::Move(double deltaX, double deltaY, double deltaZ) {

	// The robot (of which this joint is a part)
	// is being moved by the user.

	x = x + deltaX;
	y = y + deltaY;
	z = z + deltaZ;
}

void JOINT::Rotate(double angleX, double angleY, double angleZ) {
	// angle inputs are in radians; only one is non-zero

	if ( angleX != 0.0 ) {		// rotate about the x axis
		dRFromAxisAndAngle(R,1,0,0,angleX);
	}
	else if ( angleY != 0.0 ) {	// rotate about the y axis
		dRFromAxisAndAngle(R,0,1,0,angleY);
	}
	else if ( angleZ != 0.0 ) {	// rotate about the z axis
		dRFromAxisAndAngle(R,0,0,1,angleZ);
	}

	// matrix-vector multiply
	int rMax = 3;
	int cMax = 4;
	dVector3 nAxis = {0,0,0,0};
	// axisX, axisY, axisZ define the axis to which the joint is aligned 
	dVector3 oAxis = {axisX, axisY, axisZ,0};
	for ( int i=0; i<rMax; i++ ) {
		for ( int j=0; j<cMax-1; j++ ) {// last col is all zeros
			nAxis[i] += R[i*cMax+j] * oAxis[j];
		}
	}
	axisX = nAxis[0];
	axisY = nAxis[1];
	axisZ = nAxis[2];
}

void JOINT::Save(ofstream *outFile) {

	(*outFile) << obj1Index	 << " " << obj2Index  << " \n";

	(*outFile) << color[0] << " " << color[1] << " " << color[2] << " \n";

	(*outFile) << x << " " << y << " " << z << " \n";

	(*outFile) << axisX << " " << axisY << " " << axisZ << " \n";

	(*outFile) << maxFlexion << " " << maxExtension	<< " \n";

	(*outFile) << groupID <<" \n";

	(*outFile) << angle <<" \n";

	(*outFile) << jointType <<" \n";

	if ( proprioceptiveSensor )

		(*outFile) << "1 \n";
	else
		(*outFile) << "0 \n";	
}

void JOINT::Sensor_Proprioceptive_Add(void) {

	if ( !proprioceptiveSensor ) {

		proprioceptiveSensor = new PROP_SENSOR();
	}
}

int JOINT::Sensors_Number_Of(void) {

	if ( proprioceptiveSensor )

		return( 1 );

	else

		return( 0 );
}

void JOINT::Sensors_Update(void) {

	// If the robot isn't physicalized, it cannot generate
	// sensor values.
	if ( !physicalized )
		return;

	if ( proprioceptiveSensor ) {

		double actualAngle = dJointGetHingeAngle(joint);

		actualAngle = Radians_To_Degrees(actualAngle);

		actualAngle = Scale(	actualAngle,
					-ROBOT_STARFISH_JOINT_RANGE,
					+ROBOT_STARFISH_JOINT_RANGE,
					-1,1);
		// Sensor values are always scaled to [-1,1]

		proprioceptiveSensor->Update(actualAngle);
	}
}

void JOINT::Set_Color(double r, double g, double b) {

	color[0] = r;
	color[1] = g;
	color[2] = b;
}

void JOINT::Unhide(void) {

	hidden = false;
}

void JOINT::Unmark(void) {

	markIndex = -1;

	if ( Is_Unconnected() )

		Set_Color(2.0, 0.8, 0.0);
	else
		Set_Color(1.0, 0.0, 0.0);
}

// --------------------- Private methods --------------------

double JOINT::Degrees_To_Radians(double degrees) {

	return( (3.1415926536*degrees)/180.0 );
}

double JOINT::Radians_To_Degrees(double radians) {

	return( (180.0*radians)/3.1415926536 );
}

double JOINT::Rand(double min, double max) {

	// Return a random value in [min,max] with
	// a uniform distribution.

        double zeroToOne = ((double)rand()) / RAND_MAX;
        double returnVal;

        returnVal = (zeroToOne * (max-min)) + min;
        return returnVal;
}

void JOINT::Remove_From_Simulator(void) {

	dJointDestroy(joint);
	joint = NULL;

	physicalized = false;
}

double JOINT::Scale(double value, double min1, double max1,
                                       double min2, double max2) {

        if ( min1 < 0 )
                value = value - min1;
        else
                value = value + min1;

        return( (value*(max2-min2)/(max1-min1)) + min2 );
}

void JOINT::Sensors_Reset(void) {

	if ( proprioceptiveSensor )

		proprioceptiveSensor->Reset();
}

void JOINT::Draw_Geometrically(void) {
	// make the joint visible for the user

	if ( hidden ) 
		return; 

	pos[0] = x;
	pos[1] = y;
	pos[2] = z;

	dRFromZAxis(R,axisX,axisY,axisZ);

	// TBD: for now, use larger of maxFlexion and maxExtension
	double length = ( abs(maxFlexion) > abs(maxExtension) ) ?
			  abs(maxFlexion)/20.0 :
			  abs(maxExtension)/20.0;
	dsDrawCapsule(pos,R,length,JOINT_PIN_RADIUS);
}

void JOINT::Initialize(int o1Index, int o2Index, int jType,
		       double posX, double posY, double posZ,
		       double axX,  double axY,  double axZ,
		       double maxF, double maxE) {

	obj1Index = o1Index;
	obj2Index = o2Index;

	jointType = jType;

	x = posX;
	y = posY;
	z = posZ;

	axisX = axX;
	axisY = axY;
	axisZ = axZ;

	maxFlexion = maxF;
	maxExtension = maxE;

	if ( obj1Index < 0 || obj2Index < 0  )

		Set_Color(2.0, 0.8, 0.0);
}

void JOINT::Mark_Joint(void) {

	int maxMarkIndex = -1;

	for ( int i=0; i<containerRobot->numJoints; i++ ) {

		if ( containerRobot->joints[i]->markIndex > maxMarkIndex )

			maxMarkIndex = containerRobot->joints[i]->markIndex;
	}

	markIndex = maxMarkIndex++;

	Set_Color(0.4, 0.0, 0.8);

}

#endif
