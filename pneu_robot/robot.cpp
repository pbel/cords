#include "stdio.h"
#ifndef _ROBOT_CPP
#define _ROBOT_CPP

#include "robot.h"
#include "matrix.h"

extern int 	ROBOT_SANDBOX;
extern int 	ROBOT_STARFISH;
extern int	ROBOT_PNEU_QUAD;

extern int	GENERIC_JOINT;
extern int	ROBOT_BODY_JOINT;
extern int	ROBOT_LEG_JOINT;
extern int	ROBOT_BODY_TO_LEG_JOINT;

extern int	MAX_COMPONENTS;
extern int	MAX_JOINTS;

extern int	SHAPE_CYLINDER;
extern int 	SHAPE_RECTANGLE;

extern double	ROBOT_PNEU_QUAD_BODY_LENGTH;
extern double	ROBOT_PNEU_QUAD_BODY_WIDTH;
extern double	ROBOT_PNEU_QUAD_BODY_HEIGHT;
extern double	ROBOT_PNEU_QUAD_LEG_ANGLE_DEG;
extern double	ROBOT_PNEU_QUAD_LEG_LENGTH;
extern double	ROBOT_PNEU_QUAD_LEG_WIDTH_END;
extern double	ROBOT_PNEU_QUAD_LEG_WIDTH_START;
extern double	ROBOT_PNEU_QUAD_LEG_HEIGHT;
extern int	ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS;
extern int	ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;
extern int	ROBOT_PNEU_QUAD_NUM_LEGS;
extern double	ROBOT_PNEU_QUAD_CYL_RADIUS;
extern double	ROBOT_PNEU_QUAD_CYL_LENGTH;
extern double	ROBOT_PNEU_QUAD_EXT_RANGE;
extern double	ROBOT_PNEU_QUAD_LEG_FLEX_RANGE;
extern double	ROBOT_PNEU_QUAD_BODY_FLEX_RANGE;

extern double	ROBOT_SANDBOX_BOX_LENGTH;
extern double	ROBOT_SANDBOX_BOX_WIDTH;
extern double	ROBOT_SANDBOX_BOX_HEIGHT;
extern double	ROBOT_SANDBOX_CYL_RADIUS;
extern double	ROBOT_SANDBOX_CYL_LENGTH;
extern double	ROBOT_SANDBOX_JOINT_RANGE;

extern double	ROBOT_STARFISH_BODY_LENGTH;
extern double	ROBOT_STARFISH_BODY_WIDTH;
extern double	ROBOT_STARFISH_BODY_HEIGHT;
extern double	ROBOT_STARFISH_LEG_RADIUS;
extern double   ROBOT_STARFISH_LEG_LENGTH;
extern double	ROBOT_STARFISH_JOINT_RANGE;

extern int	TIME_STEPS_PER_INTERVAL;
extern int	NUM_TIME_INTERVALS;	// TBD: controller time intervals

extern double	WORST_FITNESS;

ROBOT::ROBOT(ENVIRONMENT *cE, int robotType) {

	containerEnvironment = cE;

	if ( robotType == ROBOT_STARFISH )

		Create_Starfish();

	if ( robotType == ROBOT_SANDBOX )

		Create_Sandbox();

	if ( robotType == ROBOT_PNEU_QUAD )

		Create_Pneu_Quad();

	for (int i=0;	i<numObjects;	i++)
		if ( objects[i] )
			objects[i]->containerRobot = this;

	for (int j=0;	j<numJoints;	j++)
		if ( joints[j] )
			joints[j]->containerRobot = this;

	botType = robotType;

	hidden = false;
	physicalized = false;

	activeComponent = -1;

	controller = NULL;

	sensorDifferences = 0.0;
}

ROBOT::ROBOT(ROBOT *other) {

	containerEnvironment = other->containerEnvironment;

	Initialize(other);
}

ROBOT::ROBOT(ENVIRONMENT *cE, ROBOT *other) {

	containerEnvironment = cE;

	Initialize(other);
}

ROBOT::ROBOT(ENVIRONMENT *cE, ifstream *inFile) {

	containerEnvironment = cE;

	Initialize(inFile);
}

ROBOT::~ROBOT(void) {

	if ( containerEnvironment )
		containerEnvironment = NULL;

	if ( objects ) {
		for (int i=0;	i<numObjects;	i++) {
			if ( objects[i] ) {
				delete objects[i];
				objects[i] = NULL;
			}
		}
		delete[] objects;
		objects = NULL;
	}

	if ( joints ) {
		for (int j=0;	j<numJoints;	j++) {
			if ( joints[j] ) {
				delete joints[j];
				joints[j] = NULL;
			}
		}
		delete[] joints;
		joints = NULL;
	}

	if ( controller ) {

		delete controller;
		controller = NULL;
	}
}

void ROBOT::Activate(void) {

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Activate();
}

void ROBOT::Activate_Component(void) {

	if ( activeComponent < 0 ) 
		activeComponent = 0;

	if ( (numObjects > 0) || (numJoints > 0) )
		if ( activeComponent < numObjects ) 
			objects[activeComponent]->Activate();
		else {
			joints[activeComponent - numObjects]->Activate();
//			printf("active joint number: %d, obj1: %d  obj2: %d\n",
//			       activeComponent - numObjects,
//			       joints[activeComponent-numObjects]->obj1Index,
//			       joints[activeComponent-numObjects]->obj2Index);
		}
}

void ROBOT::Active_Component_Copy(void) {

	if ( activeComponent < 0 )
		return;

	if ( activeComponent < numObjects ) {	// not a joint

		objects[numObjects] = new OBJECT(this, objects[activeComponent]);

		objects[activeComponent]->Deactivate();

		activeComponent = numObjects;

		numObjects++;

		objects[activeComponent]->Activate();

		objects[activeComponent]->Move(-0.1,0.1,0);
	}
	else {					// copying a joint

		int jointIndex = activeComponent - numObjects;

		joints[numJoints] = new JOINT(this, joints[jointIndex]);

		// require explicit connections of joint to objects
		joints[numJoints]->obj1Index = -1;
		joints[numJoints]->obj2Index = -1;

		joints[jointIndex]->Deactivate();

		activeComponent = numObjects + numJoints;

		numJoints++;

		joints[activeComponent - numObjects]->Activate();

		joints[activeComponent - numObjects]->Move(-0.1,0.1,0);
	}
}

void ROBOT::Active_Component_Delete(void) {

	if ( activeComponent < numObjects ) {	// deleting an object

		if ( numObjects > 1 ) { // don't delete the only object

			delete objects[activeComponent];

			objects[activeComponent] = NULL;

			// disconnect joints attached to the deleted component
			for (int j=0; j < numJoints; j++ ) {

				if ( joints[j]->obj1Index == activeComponent
				  || joints[j]->obj2Index == activeComponent)

					joints[j]->Disconnect();
			}

			Component_Reindex_After_Deletion();

			numObjects--;

			// change the color of any unattached joint pins
			for (int j=0; j < numJoints; j++ ) {

				if ( joints[j]->Is_Unconnected() )

					joints[j]->Set_Color(2.0, 0.8, 0.0);
			}
		}
	}
	else {					// deleting a joint

		int jointIndex = activeComponent - numObjects;

		Joint_Delete(jointIndex);
	}

	if ( activeComponent >= numObjects + numJoints) // out of range

		activeComponent = numObjects + numJoints - 1;

	if ( activeComponent >= numObjects ) 

		joints[activeComponent-numObjects]->Activate();
	else
		objects[activeComponent]->Activate();
}

void ROBOT::Active_Component_Decrement(void) {

	if ( activeComponent > 0 )
		activeComponent--;
	else 
		activeComponent = numObjects + numJoints - 1;
}

void ROBOT::Active_Component_Increment(void) {

	activeComponent++;

	if ( activeComponent >= (numObjects + numJoints) ) 
		activeComponent = 0;
}

void ROBOT::Active_Component_Move(double x, double y, double z) {

	// The component is being moved by the user.
	if ( activeComponent < numObjects ) {

		if ( objects[activeComponent]->active )
			objects[activeComponent]->Move(x,y,z);
	}

	else {
		int jointIndex = activeComponent - numObjects;

		if ( joints[jointIndex]->active )
			joints[jointIndex]->Move(x,y,z);
	}
}

void ROBOT::Active_Component_Resize(double x, double y, double z) {

	if ( (activeComponent >= numObjects) || (activeComponent < 0) )
		return;

	objects[activeComponent]->Resize(x,y,z);
}

void ROBOT::Active_Component_Rotate(double rX, double rY, double rZ) {

	if ( activeComponent < numObjects ) {

		if ( objects[activeComponent]->active )
			objects[activeComponent]->Rotate(rX,rY,rZ);
	}

	else {
		int jointIndex = activeComponent - numObjects;

		if ( joints[jointIndex]->active )
			joints[jointIndex]->Rotate(rX,rY,rZ);
	}
}

void ROBOT::Active_Joint_Range_Change(double delta) {

	if ( (activeComponent < numObjects) || (numJoints < 1) )
		return;

	int jInd = activeComponent - numObjects;

//	printf("maxFlexion = %f, maxE = %f\n",
//	       joints[jInd]->maxFlexion, joints[jInd]->maxExtension);

	joints[jInd]->maxFlexion *= delta;
	joints[jInd]->maxExtension *= delta;

	if ( joints[jInd]->maxFlexion > 0 )
		joints[jInd]->maxFlexion = -0.1;

	if ( joints[jInd]->maxExtension < 0 )
		joints[jInd]->maxExtension = 0.1;
}

int ROBOT::Components_Number_Of(void) {

	return( numObjects + numJoints );
}

void ROBOT::Connect_Robot_Joint(void) {

	Connect_Joint();
}

void ROBOT::Deactivate_Component() {

	if ( activeComponent < numObjects ) {

		if ( objects[activeComponent]->active )
			objects[activeComponent]->Deactivate();
	}

	else {
		int jointIndex = activeComponent - numObjects;

		if ( joints[jointIndex]->active )
			joints[jointIndex]->Deactivate();
	}
}

void ROBOT::Deactivate(void) {

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Deactivate();

	for (int i=0;	i<numJoints;	i++)

		if ( joints[i] )
			joints[i]->Deactivate();
}

void ROBOT::Draw(void) {

	if ( hidden )
		return;

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Draw();

	for (int i=0;	i<numJoints;	i++)

		if ( joints[i] )
			joints[i]->Draw();
}

double ROBOT::Fitness_Get(ROBOT *targetRobot) {

/*
//	if ( controller->inAFixedAttractor )
//		return( WORST_FITNESS );

	double fitness = 0.0;

	double diff = Sensors_Get_Total_Differences(targetRobot);

	//double diff = Sensors_Get_Largest_Difference(targetRobot);

	fitness = -diff;
	
//	fitness = Sensors_Get_Total();

	return( fitness );
*/

	return( -sensorDifferences );
}

int ROBOT::Get_Num_Motor_Groups(void) {

	int jointGrps[MAX_JOINTS];

	for (int i=0; i<MAX_JOINTS; i++)
		jointGrps[i] = 0;

	int sum = 0;

	for (int i=0; i<numJoints; i++)

		jointGrps[joints[i]->groupID] = 1;

	for (int i=0; i<MAX_JOINTS; i++) 

		sum += jointGrps[i];

	//printf("sum: %d\n", sum);

	return sum;
}

int ROBOT::Get_Num_Time_Steps(void) {

	return NUM_TIME_INTERVALS; // TBD: could be under evolutionary control 

//	return controller->numTimeSteps; // TBD: crashes with this line
	// TBD: check whether we're loading from a file, and use that value if
	// we are; if not, use the constant; could have another var indicating
	// whether we're loading from a file

}

void ROBOT::Group_Robot_Joints(void) {

	Group_Joints();
}

int  ROBOT::Has_Stopped(void) {

	if ( !controller )
		return( true );

	return( controller->inAFixedAttractor );
}

void ROBOT::Hide(void) {

	hidden = true;

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Hide();

	for (int i=0;	i<numJoints;	i++)

		if ( joints[i] )
			joints[i]->Hide();
}

void ROBOT::Hide_Robot_Joints(void) {

	for (int i=0;	i<numJoints;	i++)

		if ( joints[i] )
			joints[i]->Hide();
}

int  ROBOT::In_Simulator(void) {

	return( physicalized );
}

int ROBOT::Joint_Is_Unattached(void) {

	int jointUnattached = false;

	for ( int j=0; j<numJoints; j++ ) {

		if ( joints[j]->Is_Unconnected() )

			jointUnattached = true; 
	}
	return jointUnattached;
}

void ROBOT::Label(CONTROLLER *genome, int environmentIndex) {

	if ( controller )
		delete controller;

	controller = new CONTROLLER(genome);
}

void ROBOT::Label_From_File(CONTROLLER *genome, int environmentIndex, char* fileName) {

	if ( controller )
		delete controller;

	controller = new CONTROLLER(genome);

	controller->Load_From_File(fileName);
}

void ROBOT::Make_Incorporeal(void) {

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Make_Incorporeal();

	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )
			joints[j]->Make_Incorporeal();

	physicalized = false;
}

void ROBOT::Make_Physical(dWorldID world, dSpaceID space) {

	// Add the robot to the physical simulator.

	if ( physicalized )
		return;

	physicalized = true;

//	dReal bodyMult = 1/(dReal)ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS;
//	dReal legMult  = 1/(dReal)ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;
	dReal bodyMult = 1/(dReal)ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS;
	dReal legMult  = 1/(dReal)ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;

	//printf("botType = %d\n",botType);

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			if ( botType != ROBOT_PNEU_QUAD )

				objects[i]->Make_Physical(world,space);

			else if ( i < ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS )

				objects[i]->Make_Physical(world,space,bodyMult);

			else
				objects[i]->Make_Physical(world,space,legMult);

	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )
			joints[j]->Make_Physical(world);
}

int  ROBOT::Motors_Number_Of(void) {

	return( numJoints );
	// Assumes all joints are motorized.
}

void ROBOT::Move(int timeStep) {

	// The robot is moving itself.

	// The robot cannot move itself it is not physical.
	if ( !physicalized )
		return;

//	if ( timeStep==0 )
	Controller_Set_Sensors();

	// for evolving controllers
	Controller_Update(timeStep);

	// calculate the row of the motors matrix to access
	int timeSeqNum = (timeStep/TIME_STEPS_PER_INTERVAL) % controller->numTimeSteps;

	Actuate_Motors(timeSeqNum);

//	Actuate_Motors();

	Sensors_Touch_Clear();
}

void ROBOT::Move(double x, double y, double z) {

	// The robot is being moved by the user.

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Move(x,y,z);

	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )
			joints[j]->Move(x,y,z);
}

void ROBOT::Save(ofstream *outFile) {

	(*outFile) << numObjects << "\n";

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Save(outFile);

	(*outFile) << numJoints << "\n";

	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )
			joints[j]->Save(outFile);

	(*outFile) << botType << "\n"; 
}

void ROBOT::Mark_Component(void) {

	if ( activeComponent < 0 )
		return;

	if ( activeComponent < numObjects )

		objects[activeComponent]->Mark();
	else

		joints[activeComponent-numObjects]->Mark();
}

double ROBOT::Sensor_Sum(void) {

	return( Sensors_Get_Total() );
}

void ROBOT::Sensors_Add_Difference(ROBOT *other) {

	sensorDifferences = sensorDifferences + 

				Sensors_In_Objects_Total_Differences(other);
}

int ROBOT::Sensors_Number_Of(void) {

	int numSensors = 0;

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			numSensors = 	numSensors + 
					objects[i]->Sensors_Number_Of();


	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )
			numSensors = 	numSensors + 
					joints[j]->Sensors_Number_Of();

	return( numSensors );
}

void ROBOT::Sensors_Update(void) {

	// Light sensors already updated during the
	// last drawing of the robot.
	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Sensors_Update();

	// Touch sensors updated by nearCallback function.

	// Update all of the proprioceptive sensors.
	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )
			joints[j]->Sensors_Update();
}

void ROBOT::Sensors_Write(void) {

	Sensors_In_Objects_Write();

	Sensors_In_Joints_Write();

	printf("\n");
}

void ROBOT::Set_Color(double r, double g, double b) {

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Set_Color(r,g,b);

	for (int i=0;	i<numJoints;	i++) {

		if ( joints[i] )
			joints[i]->Set_Color(r,g,b);

		// special-case unconnected joints
		if ( joints[i]->Is_Unconnected() )
			joints[i]->Set_Color(2.0, 0.8, 0.0);
	}
}

void ROBOT::Unattached_Joints_Unhide(void) {

	for ( int j=0; j<numJoints; j++ ) {

		if ( joints[j]->Is_Unconnected() )

			joints[j]->Unhide();
	}
}

void ROBOT::Unhide(void) {

	hidden = false;

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )
			objects[i]->Unhide();

	for (int i=0;	i<numJoints;	i++)

		if ( joints[i] )
			joints[i]->Unhide();
}

void ROBOT::Unmark_All(void) {

	for (int i=1; i < numObjects; i++)
		objects[i]->Unmark();

	for (int i=0;	i<numJoints;	i++)
		if ( joints[i] )
			joints[i]->Unmark();
}

void ROBOT::Unmark_Component(void) {

	if ( activeComponent < 0 )
		return;

	if ( activeComponent < numObjects )

		objects[activeComponent]->Unmark();
	else

		joints[activeComponent-numObjects]->Unmark();
}

// --------------------- Private methods ------------------------

/*
void ROBOT::Actuate_Motors(void) {

	for (int j=0;	j<numJoints;	j++) {

		double motorNeuronValue = 

		controller->Get_Motor_Neuron_Value(j);

		if ( joints[j] )
			joints[j]->Move(motorNeuronValue);
	}
}
*/

void ROBOT::Actuate_Motors(int timeSeqNum) {

	if ( !joints[0] ) 
		return;

	int jG = -1;	// joint group ID

//	double bodyCurve = 90.0; // in degrees, for complete body
//	double legCurve  = 50.0;  // in degrees
	double bodyCurve = 50.0; // in degrees, for complete body
	double legCurve  = 90.0;  // in degrees

	double bodyAngle = bodyCurve / ( (double)ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS - 1.0 );
	double legAngle  = legCurve  / ( (double)ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS  - 1.0 );

	//printf("(robot.cpp) bodyAngle: %f, legAngle: %f\n",bodyAngle,legAngle);

	// joint group assignments
	// 0: body
	// 1: LF leg
	// 2: RF leg
	// 3: LR leg
	// 4: RR leg
	// 5: all leg-to-body joints (static)

	for ( int i=0; i<numJoints; i++ ) {

		jG = joints[i]->groupID;

		if ( jG < 0 || jG > 5 ) // TBD: not generalizable!
			printf("robot.cpp, Actuate_Motors: joint group out of range.\n");

		if ( controller->Get_Motor_Neuron_Value(jG, timeSeqNum) < 1.0 ) {

			joints[i]->Move(botType,
					Degrees_To_Radians(ROBOT_PNEU_QUAD_EXT_RANGE));
		}
		else {
			if ( jG == 0 ) 				// body joint

				joints[i]->Move(botType,
						Degrees_To_Radians(bodyAngle));

			if ( jG > 0 && jG < 5 )			// leg joint

				joints[i]->Move(botType, 
						Degrees_To_Radians(legAngle));
			if ( jG == 5 )				// leg-to-body
				joints[i]->Move(botType,
						Degrees_To_Radians(ROBOT_PNEU_QUAD_EXT_RANGE));
		}
		//printf("jG: %d\tjoint %d angle: %0.3f\n",joints[i]->groupID, i, joints[i]->angle);
	}
	// map the motors matrix to the joint groups 
//	for (int j=0;	j<numJoints;	j++) {

}

void ROBOT::Component_Reindex_After_Deletion(void) {

	// deleted comp was not the last object; re-index objects
	if ( activeComponent < numObjects-1 ) {

		for (int i=activeComponent+1; i<numObjects; i++) {
			objects[i-1] = objects[i];

			// re-index joints
			for (int j=0; j < numJoints; j++ ) {

				if ( joints[j]->obj1Index == i)
					joints[j]->obj1Index = i-1;

				if ( joints[j]->obj2Index == i )
					joints[j]->obj2Index = i-1;
			}
		}
	}
}

void ROBOT::Connect_Joint(void) {

	// connect the last two marked objects with the joint

	if ( activeComponent < numObjects ) // the active comp must be a joint
		return; 

	int jInd =  activeComponent - numObjects;

	int i;
	int maxMarkIndex = -1;
	int maxIndex = -1;
	int penMarkIndex = -1;
	int penIndex = -1;

	for ( i=0; i < numObjects; i++ ) {
		if ( objects[i]->markIndex > maxMarkIndex ) {
			penMarkIndex = maxMarkIndex;
			penIndex = maxIndex;
			maxMarkIndex = objects[i]->markIndex;
			maxIndex = i;
		} else {
			if ( objects[i]->markIndex > penMarkIndex ) {
				penMarkIndex = objects[i]->markIndex;
				penIndex = i;
			}
		}
	}

	joints[jInd] -> obj1Index = penIndex; // penultimate marked is object 1
	joints[jInd] -> obj2Index = maxIndex; // last marked is object 2

	if ( joints[jInd] -> obj1Index < 0 || joints[jInd] -> obj2Index < 0 ) {
		joints[jInd] -> obj1Index = -1;
		joints[jInd] -> obj2Index = -1;
		return;
	}

//	printf("next-to-last marked: %d, last marked: %d\n",
//	       penIndex, maxIndex);
//	printf("joint index: %d  obj1: %d  obj2: %d\n",
//	       jInd,penIndex,maxIndex);

//	printf("robot.cpp: joint %d: obj1 = %d;\tobj2 = %d\n",
//	       jInd, joints[jInd] -> obj1Index,joints[jInd] -> obj2Index);

	joints[jInd] -> maxFlexion   = -ROBOT_SANDBOX_JOINT_RANGE;
	joints[jInd] -> maxExtension = +ROBOT_SANDBOX_JOINT_RANGE;

	objects[joints[jInd]->obj1Index]->Unmark();
	objects[joints[jInd]->obj2Index]->Unmark();

	joints[jInd]->Set_Color(1.0, 0.0, 0.0);
}

// TBD: move hard-coded robot definitions to separate files 
// TBD: (starfish, pneu_quad) 

void ROBOT::Create_Pneu_Quad(void) {

	Create_Pneu_Quad_Objects();

	Create_Pneu_Quad_Joints();
}

void ROBOT::Create_Pneu_Quad_Joints(void) {

	joints = new JOINT * [MAX_JOINTS];

	for (int j=0;	j<numJoints;	j++)
		joints[j] = NULL;

	int numBodySegments = ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS;
	int numLegSegments = ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;
	int numLegs = ROBOT_PNEU_QUAD_NUM_LEGS;

	numJoints = numBodySegments - 1 + numLegs * numLegSegments;
	printf("numJoints = %d\n",numJoints);

	// Need the object index number for the objects that the joint connects.
	// The following requires the order of construction to be: 
	// 0. body
	// 1. LF leg
	// 2. RF leg
	// 3. LR leg
	// 4. RR leg
	// body is built from the lower legs to the upper legs
	// legs are built from the body to the end of the leg
	int bodyIndxStart = 0;
	int LF_leg_IndxStart = ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS;
	int RF_leg_IndxStart = ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS + ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;
	int LR_leg_IndxStart = ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS +
		2 * ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;
	int RR_leg_IndxStart = ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS +
		3 * ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;

	// body joints
	double bodyAngleDeg = 0;
	double bodyAngleRad = Degrees_To_Radians(bodyAngleDeg); // angle from y axis

	float bodyXincr = 0;
	float bodyYincr = ROBOT_PNEU_QUAD_BODY_LENGTH /((float)ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS - 1);

	int jointCount = 0;

	int jointGroupID = 0;
	int bodyRef = bodyIndxStart; // index of first body segment
	for(int i=0; i<numBodySegments-1; i++) {
		//printf("body_IndxStart = %d\ni = %d\n",bodyRef,i);
		joints[i+jointCount] = new JOINT(this, bodyRef + i, bodyRef + i + 1,
						 ROBOT_BODY_JOINT,
						 objects[bodyRef+i]->x,
						 objects[bodyRef+i]->y + 0.5 * bodyYincr,
						 ROBOT_PNEU_QUAD_CYL_RADIUS,
						 1, 0, 0,
						 ROBOT_PNEU_QUAD_BODY_FLEX_RANGE,
						 ROBOT_PNEU_QUAD_EXT_RANGE,
						 jointGroupID);
	}
	jointCount += numBodySegments-1;

	// leg joints
	double legAngleRad = Degrees_To_Radians(ROBOT_PNEU_QUAD_LEG_ANGLE_DEG); // angle from y axis
	double sin_legAngle = sin(legAngleRad);
	double cos_legAngle = cos(legAngleRad);

	float legXincr = (ROBOT_PNEU_QUAD_LEG_LENGTH * sin_legAngle)/((float)numLegSegments - 1);
	float legYincr = (ROBOT_PNEU_QUAD_LEG_LENGTH * cos_legAngle)/((float)numLegSegments - 1);

	float legBodyXincr = 0.25 * ROBOT_PNEU_QUAD_BODY_WIDTH;
	float legBodyYincr = 0.5  * ROBOT_PNEU_QUAD_LEG_WIDTH_START * sin_legAngle; 

	// adjust z-axis of joint pin for better foot contact
	dReal ffact = Degrees_To_Radians(-20);

	// LF leg
	jointGroupID = 1;
	int legRef = LF_leg_IndxStart;
	for(int i=0; i<numLegSegments-1; i++) {
		//printf("leg_IndxStart = %d\ni = %d\n",legRef,i);
		joints[i+jointCount] = new JOINT(this, legRef + i, legRef + i + 1,
						 ROBOT_LEG_JOINT,
						 objects[legRef+i]->x - 0.5 * legXincr,
						 objects[legRef+i]->y + 0.5 * legYincr,
						 ROBOT_PNEU_QUAD_CYL_RADIUS,
//						 0.5 * M_PI - legAngleRad, legAngleRad, 0,
//						 0.5 * M_PI - legAngleRad - ffact, legAngleRad - ffact, 0,
						 1, 0, 0,
						 ROBOT_PNEU_QUAD_LEG_FLEX_RANGE,
						 ROBOT_PNEU_QUAD_EXT_RANGE,
						 jointGroupID);
	}
	jointCount += numLegSegments-1;

	// LF leg to body
	int LF_to_body_index = jointCount;
	jointGroupID = 5;
	joints[LF_to_body_index] = new JOINT(this, numBodySegments-1, LF_leg_IndxStart,
					     ROBOT_BODY_TO_LEG_JOINT,
					     objects[numBodySegments-1]->x - legBodyXincr, 
					     objects[numBodySegments-1]->y + legBodyYincr,
					     ROBOT_PNEU_QUAD_CYL_RADIUS,
					     0.5 * M_PI - legAngleRad, legAngleRad, 0,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     jointGroupID);
	jointCount += 1;

	// RF leg
	jointGroupID = 2;
	legRef = RF_leg_IndxStart;
	for(int i=0; i<numLegSegments-1; i++) {
		//printf("leg_IndxStart = %d\ni = %d\n",legRef,i);
		joints[i+jointCount] = new JOINT(this, legRef + i + 1, legRef + i,
						 ROBOT_LEG_JOINT,
						 objects[legRef+i]->x + 0.5 * legXincr,
						 objects[legRef+i]->y + 0.5 * legYincr,
						 ROBOT_PNEU_QUAD_CYL_RADIUS,
//						 -(0.5 * M_PI - legAngleRad), legAngleRad, 0,
//						 -(0.5 * M_PI - legAngleRad - ffact), legAngleRad - ffact, 0,
						 -1, 0, 0,
						 ROBOT_PNEU_QUAD_LEG_FLEX_RANGE,
						 ROBOT_PNEU_QUAD_EXT_RANGE,
						 jointGroupID);
	}
	jointCount += numLegSegments-1;

	// RF leg to body
	int RF_to_body_index = jointCount;
	jointGroupID = 5;
	joints[RF_to_body_index] = new JOINT(this, numBodySegments-1, RF_leg_IndxStart,
					     ROBOT_BODY_TO_LEG_JOINT,
					     objects[numBodySegments-1]->x + legBodyXincr,
					     objects[numBodySegments-1]->y + legBodyYincr,
					     ROBOT_PNEU_QUAD_CYL_RADIUS,
					     -(0.5 * M_PI - legAngleRad), legAngleRad, 0,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     jointGroupID);
	jointCount += 1;

	// LR leg
	jointGroupID = 3;
	legRef = LR_leg_IndxStart;
	for(int i=0; i<numLegSegments-1; i++) {
		//printf("leg_IndxStart = %d\ni = %d\n",legRef,i);
		joints[i+jointCount] = new JOINT(this, legRef + i, legRef + i + 1,
						 ROBOT_LEG_JOINT,
						 objects[legRef+i]->x - 0.5 * legXincr,
						 objects[legRef+i]->y - 0.5 * legYincr,
						 ROBOT_PNEU_QUAD_CYL_RADIUS,
//						 -(0.5 * M_PI - legAngleRad), legAngleRad, 0,
//						 -(0.5 * M_PI - legAngleRad - ffact), legAngleRad - ffact, 0,
						 -1, 0, 0,
						 ROBOT_PNEU_QUAD_LEG_FLEX_RANGE,
						 ROBOT_PNEU_QUAD_EXT_RANGE,
						 jointGroupID);
	}
	jointCount+=numLegSegments-1;

	// LR leg to body
	int LR_to_body_index = jointCount;
	jointGroupID = 5;
	joints[LR_to_body_index] = new JOINT(this, 0, LR_leg_IndxStart,
					     ROBOT_BODY_TO_LEG_JOINT,
					     objects[0]->x - legBodyXincr,
					     objects[0]->y - legBodyYincr,
					     ROBOT_PNEU_QUAD_CYL_RADIUS,
					     -(0.5 * M_PI - legAngleRad), legAngleRad, 0,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     jointGroupID);
	jointCount += 1;

	// RR leg
	jointGroupID = 4;
	legRef = RR_leg_IndxStart;
	for(int i=0; i<numLegSegments-1; i++) {
		//printf("leg_IndxStart = %d\ni = %d\n",legRef,i);
		joints[i+jointCount] = new JOINT(this, legRef + i + 1, legRef + i,
						 ROBOT_LEG_JOINT,
						 objects[legRef+i]->x + 0.5 * legXincr,
						 objects[legRef+i]->y - 0.5 * legYincr,
						 ROBOT_PNEU_QUAD_CYL_RADIUS,
//						 0.5 * M_PI - legAngleRad, legAngleRad, 0,
//						 0.5 * M_PI - legAngleRad - ffact, legAngleRad - ffact, 0,
						 1, 0, 0,
						 ROBOT_PNEU_QUAD_LEG_FLEX_RANGE,
						 ROBOT_PNEU_QUAD_EXT_RANGE,
						 jointGroupID);
	}
	jointCount+=numLegSegments-1;

	// RR leg to body
	int RR_to_body_index = jointCount;
	jointGroupID = 5;
	joints[RR_to_body_index] = new JOINT(this, 0, RR_leg_IndxStart,
					     ROBOT_BODY_TO_LEG_JOINT,
					     objects[0]->x + legBodyXincr,
					     objects[0]->y - legBodyYincr,
					     ROBOT_PNEU_QUAD_CYL_RADIUS,
					     0.5 * M_PI - legAngleRad, legAngleRad, 0,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     ROBOT_PNEU_QUAD_EXT_RANGE,
					     jointGroupID);
	jointCount += 1;
	printf("final jointCount = %d\n",jointCount);

	for (int j=0;j<numJoints;j++)

 		joints[j]->Sensor_Proprioceptive_Add();

}

void ROBOT::Create_Pneu_Quad_Objects(void) {

	// allowable parts for user-created robot

	int numBodySegments = ROBOT_PNEU_QUAD_NUM_BODY_SEGMENTS;
	int numLegSegments = ROBOT_PNEU_QUAD_NUM_LEG_SEGMENTS;
	int numLegs = ROBOT_PNEU_QUAD_NUM_LEGS;

	printf("numBodySeg = %d; numLegSeg = %d, numLegs = %d\n",numBodySegments,numLegSegments,numLegs);

	numObjects = numBodySegments + numLegs * numLegSegments;

	objects = new OBJECT * [MAX_COMPONENTS];
	for (int i=0;	i<MAX_COMPONENTS;	i++)
		objects[i] = NULL;

	double legAngleRad = Degrees_To_Radians(ROBOT_PNEU_QUAD_LEG_ANGLE_DEG); // angle from y axis
	double sin_legAngle = sin(legAngleRad);
	double cos_legAngle = cos(legAngleRad);

	// create float arrays that define the position of the outline of the
	// robot part to build (e.g., leg or body). 
	// Here, the center of the cylinder defining each side

	// body reference points
	float body_upper_ctr[2] = { 0,  0.5 * ROBOT_PNEU_QUAD_BODY_LENGTH };
	float body_lower_ctr[2] = { 0, -0.5 * ROBOT_PNEU_QUAD_BODY_LENGTH };

	// leg reference points
	// left front leg 
	// calc leg start center relative to body upper center
	float LF_leg_start_ctr[2] = { body_upper_ctr[0] - 0.5 * ROBOT_PNEU_QUAD_BODY_WIDTH 
				      + cos_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START, 
				      body_upper_ctr[1] + ROBOT_PNEU_QUAD_CYL_RADIUS
				      + sin_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START };
	// calc leg upper center relative to leg lower center
	float LF_leg_end_ctr[2]   = { LF_leg_start_ctr[0]
				      - sin_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH,
				      LF_leg_start_ctr[1]
				      + cos_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH };

	// right front leg
	float RF_leg_start_ctr[2] = { body_upper_ctr[0] + 0.5 * ROBOT_PNEU_QUAD_BODY_WIDTH 
				      - cos_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START, 
				      body_upper_ctr[1] + ROBOT_PNEU_QUAD_CYL_RADIUS
				      + sin_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START };

	float RF_leg_end_ctr[2]   = { RF_leg_start_ctr[0]
				      + sin_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH,
				      RF_leg_start_ctr[1]
				      + cos_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH };

	// left rear leg
	float LR_leg_start_ctr[2] = { body_lower_ctr[0] - 0.5 * ROBOT_PNEU_QUAD_BODY_WIDTH 
				      + cos_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START, 
				      body_lower_ctr[1] - ROBOT_PNEU_QUAD_CYL_RADIUS
				      - sin_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START };

	float LR_leg_end_ctr[2]   = { LR_leg_start_ctr[0]
				      - sin_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH,
				      LR_leg_start_ctr[1]
				      + cos_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH };

	// right rear leg
	float RR_leg_start_ctr[2] = { body_lower_ctr[0] + 0.5 * ROBOT_PNEU_QUAD_BODY_WIDTH 
				      - cos_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START, 
				      body_lower_ctr[1] - ROBOT_PNEU_QUAD_CYL_RADIUS
				      - sin_legAngle * 0.5 * ROBOT_PNEU_QUAD_LEG_WIDTH_START };

	float RR_leg_end_ctr[2]   = { RF_leg_start_ctr[0]
				      - sin_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH,
				      RF_leg_start_ctr[1] 
				      + cos_legAngle * ROBOT_PNEU_QUAD_LEG_LENGTH };

	float legXincr = (ROBOT_PNEU_QUAD_LEG_LENGTH * sin_legAngle)/((float)numLegSegments - 1);
	float legYincr = (ROBOT_PNEU_QUAD_LEG_LENGTH * cos_legAngle)/((float)numLegSegments - 1);
	float legWidthDecr = (ROBOT_PNEU_QUAD_LEG_WIDTH_START-ROBOT_PNEU_QUAD_LEG_WIDTH_END)/
				((float)numLegSegments - 1);
//	printf("legXincr = %f, legYincr = %f\n",legXincr, legYincr);

	// body
	float bodyXincr = ROBOT_PNEU_QUAD_BODY_LENGTH/((float)numBodySegments - 1);

	int objectCount = 0;

	for (int j0=0; j0<numBodySegments; j0++) {
		objects[j0] = new OBJECT(SHAPE_CYLINDER, 
					 ROBOT_PNEU_QUAD_CYL_RADIUS,
					 ROBOT_PNEU_QUAD_BODY_WIDTH,
					 body_lower_ctr[0],
					 body_lower_ctr[1] 
					 + j0 * bodyXincr,
					 ROBOT_PNEU_QUAD_CYL_RADIUS,
					 1.0, 0.0, 0);
	}
	objectCount += numBodySegments;

	// left front leg
	for (int j1=0; j1<numLegSegments; j1++) {
		objects[j1+objectCount] = new OBJECT(SHAPE_CYLINDER, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
						     ROBOT_PNEU_QUAD_LEG_WIDTH_START - j1 * legWidthDecr,
						     LF_leg_start_ctr[0] - j1 * legXincr, 
						     LF_leg_start_ctr[1] + j1 * legYincr, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
//						     0.5 * M_PI - legAngleRad, legAngleRad, 0);
						     1, 0, 0);
	}
	objectCount += numLegSegments;

	// right front leg
	for (int j2=0; j2<numLegSegments; j2++) {
		objects[j2+objectCount] = new OBJECT(SHAPE_CYLINDER, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
						     ROBOT_PNEU_QUAD_LEG_WIDTH_START - j2 * legWidthDecr,
						     RF_leg_start_ctr[0] + j2 * legXincr, 
						     RF_leg_start_ctr[1] + j2 * legYincr, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
//						     -(0.5 * M_PI - legAngleRad), legAngleRad, 0);
						     1, 0, 0);
	}
	objectCount += numLegSegments;

	// left rear leg
	for (int j3=0; j3<numLegSegments; j3++) {
		objects[j3+objectCount] = new OBJECT(SHAPE_CYLINDER, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
						     ROBOT_PNEU_QUAD_LEG_WIDTH_START - j3 * legWidthDecr,
						     LR_leg_start_ctr[0] - j3 * legXincr, 
						     LR_leg_start_ctr[1] - j3 * legYincr, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
//						     0.5 * M_PI - legAngleRad, -legAngleRad, 0);
						     1, 0, 0);
	}
	objectCount += numLegSegments;

	// right rear leg
	for (int j4=0; j4<numLegSegments; j4++) {
		objects[j4+objectCount] = new OBJECT(SHAPE_CYLINDER, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
						     ROBOT_PNEU_QUAD_LEG_WIDTH_START - j4 * legWidthDecr,
						     RR_leg_start_ctr[0] + j4 * legXincr, 
						     RR_leg_start_ctr[1] - j4 * legYincr, 
						     ROBOT_PNEU_QUAD_CYL_RADIUS,
//						     0.5 * M_PI - legAngleRad, legAngleRad, 0);
						     1, 0, 0);
	}
	objectCount += numLegSegments;

	// automatically add light and touch sensor to all components
	for (int i=0;i<numObjects;i++) {

		objects[i]->Sensor_Light_Add();

		objects[i]->Sensor_Touch_Add();
	}
}

void ROBOT::Create_Sandbox(void) {

	// create robot parts to be copied/assembled by user

	Create_Sandbox_Objects();

	Create_Sandbox_Joints();

}

void ROBOT::Create_Sandbox_Joints(void) {

	numJoints = 1;

	joints = new JOINT * [MAX_JOINTS];

	for (int j=0;	j<numJoints;	j++)
		joints[j] = NULL;

	joints[0] = new JOINT(this,-1, -1, GENERIC_JOINT,
			      0.0, 
			      ROBOT_SANDBOX_BOX_LENGTH/2.0,
			      0.4,
			      1.0, 0.0, 0.0,
			      -ROBOT_SANDBOX_JOINT_RANGE,
			      +ROBOT_SANDBOX_JOINT_RANGE);

	for (int j=0;j<numJoints;j++)

		joints[j]->Sensor_Proprioceptive_Add();
}

void ROBOT::Create_Sandbox_Objects(void) {

	// allowable parts for user-created robot
	numObjects = 2;

	objects = new OBJECT * [MAX_COMPONENTS];
	for (int i=0;	i<MAX_COMPONENTS;	i++)
		objects[i] = NULL;

	// box
  	objects[0] = new OBJECT(SHAPE_RECTANGLE, 
				ROBOT_SANDBOX_BOX_LENGTH,
				ROBOT_SANDBOX_BOX_WIDTH,
				ROBOT_SANDBOX_BOX_HEIGHT,
				0.0,
				0.0,
				0.2,
				0,0,1);

	// cylinders
  	objects[1] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_SANDBOX_CYL_RADIUS,
				ROBOT_SANDBOX_CYL_LENGTH,
				0,
				1.2,
				0.2,
				0.0,1.0,0.0);

	// automatically add light and touch sensor to all components
	for (int i=0;i<numObjects;i++) {

		objects[i]->Sensor_Light_Add();

		objects[i]->Sensor_Touch_Add();
	}
}

void ROBOT::Create_Starfish(void) {

	Create_Starfish_Objects();

	Create_Starfish_Joints();
}

void ROBOT::Create_Starfish_Joints(void) {

	// Four joints connecting each lower and upper leg, and
	// four joints connecting each leg to the main body.
	numJoints = 4 + 4;

	joints = new JOINT * [numJoints];

	for (int j=0;	j<numJoints;	j++)
		joints[j] = NULL;

	// Attach the left upper and lower legs.
	joints[0] = new JOINT(this,1,5,GENERIC_JOINT,
				-ROBOT_STARFISH_BODY_LENGTH/2.0
				-ROBOT_STARFISH_LEG_LENGTH,
				0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				0,1,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	// Attach the right upper and lower legs.
	joints[1] = new JOINT(this,2,6,GENERIC_JOINT,
				+ROBOT_STARFISH_BODY_LENGTH/2.0
				+ROBOT_STARFISH_LEG_LENGTH,
				0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				0,-1,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	// Attach the forward upper and lower legs.
	joints[2] = new JOINT(this,3,7,GENERIC_JOINT,
				0,
				+ROBOT_STARFISH_BODY_LENGTH/2.0
				+ROBOT_STARFISH_LEG_LENGTH,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				1,0,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	// Attach the back upper and lower legs.
	joints[3] = new JOINT(this,4,8,GENERIC_JOINT,
				0,
				-ROBOT_STARFISH_BODY_LENGTH/2.0
				-ROBOT_STARFISH_LEG_LENGTH,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				-1,0,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	// Attach main body and the left upper leg.
	joints[4] = new JOINT(this,0,1,GENERIC_JOINT,
				-ROBOT_STARFISH_BODY_LENGTH/2.0,
				0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				0,1,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	// Attach main body and the right upper leg.
	joints[5] = new JOINT(this,0,2,GENERIC_JOINT,
				+ROBOT_STARFISH_BODY_LENGTH/2.0,
				0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				0,-1,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	// Attach main body and the forward upper leg.
	joints[6] = new JOINT(this,0,3,GENERIC_JOINT,
				0,
				+ROBOT_STARFISH_BODY_LENGTH/2.0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				1,0,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	// Attach main body and the back upper leg.
	joints[7] = new JOINT(this,0,4,GENERIC_JOINT,
				0,
				-ROBOT_STARFISH_BODY_LENGTH/2.0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				-1,0,0,
				-ROBOT_STARFISH_JOINT_RANGE,+ROBOT_STARFISH_JOINT_RANGE);

	for (int j=0;j<numJoints;j++)

		joints[j]->Sensor_Proprioceptive_Add();
}

void ROBOT::Create_Starfish_Objects(void) {

	// One main body, four upper legs and four lower legs
	numObjects = 1 + 4 + 4;

	objects = new OBJECT * [numObjects];

	for (int i=0;	i<numObjects;	i++)
		objects[i] = NULL;

	// Main body
  	objects[0] = new OBJECT(SHAPE_RECTANGLE, 
				ROBOT_STARFISH_BODY_LENGTH,
				ROBOT_STARFISH_BODY_WIDTH,
				ROBOT_STARFISH_BODY_HEIGHT,
				0,
				0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				0,0,1);
	
	// Left upper leg
  	objects[1] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				-ROBOT_STARFISH_BODY_LENGTH/2.0
				-ROBOT_STARFISH_LEG_LENGTH/2.0,
				0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				-1,0,0);

	// Right upper leg
  	objects[2] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				+ROBOT_STARFISH_BODY_LENGTH/2.0
				+ROBOT_STARFISH_LEG_LENGTH/2.0,
				0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				+1,0,0);

	// Forward upper leg
  	objects[3] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				0,
				+ROBOT_STARFISH_BODY_LENGTH/2.0
				+ROBOT_STARFISH_LEG_LENGTH/2.0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				0,+1,0);

	// Back upper leg
  	objects[4] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				0,
				-ROBOT_STARFISH_BODY_LENGTH/2.0
				-ROBOT_STARFISH_LEG_LENGTH/2.0,
				ROBOT_STARFISH_LEG_LENGTH
				+ROBOT_STARFISH_LEG_RADIUS,
				0,-1,0);

	// Left lower leg
  	objects[5] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				-ROBOT_STARFISH_BODY_LENGTH/2.0
				-ROBOT_STARFISH_LEG_LENGTH,
				0,
				ROBOT_STARFISH_LEG_LENGTH/2.0
				+ROBOT_STARFISH_LEG_RADIUS,
				0,0,+1);

	// Right lower leg
  	objects[6] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				+ROBOT_STARFISH_BODY_LENGTH/2.0
				+ROBOT_STARFISH_LEG_LENGTH,
				0,
				ROBOT_STARFISH_LEG_LENGTH/2.0
				+ROBOT_STARFISH_LEG_RADIUS,
				0,0,+1);

	// Forward lower leg
  	objects[7] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				0,
				+ROBOT_STARFISH_BODY_LENGTH/2.0
				+ROBOT_STARFISH_LEG_LENGTH,
				ROBOT_STARFISH_LEG_LENGTH/2.0
				+ROBOT_STARFISH_LEG_RADIUS,
				0,0,+1);

	// Back lower leg
  	objects[8] = new OBJECT(SHAPE_CYLINDER, 
				ROBOT_STARFISH_LEG_RADIUS,
				ROBOT_STARFISH_LEG_LENGTH,
				0,
				-ROBOT_STARFISH_BODY_LENGTH/2.0
				-ROBOT_STARFISH_LEG_LENGTH,
				ROBOT_STARFISH_LEG_LENGTH/2.0
				+ROBOT_STARFISH_LEG_RADIUS,
				0,0,+1);

	for (int i=0;i<numObjects;i++)

		objects[i]->Sensor_Light_Add();

	objects[5]->Sensor_Touch_Add();
	objects[6]->Sensor_Touch_Add();
	objects[7]->Sensor_Touch_Add();
	objects[8]->Sensor_Touch_Add();
}

double ROBOT::Degrees_To_Radians(double degrees) {

	return( (3.1415926536*degrees)/180.0 );
}

bool  ROBOT::File_Exists(char *fileName) {

	ifstream ifile(fileName);
	return ifile;
}

int  ROBOT::File_Index_Next_Available(void) {

	int fileIndex = 0;
	char fileName[100];
	sprintf(fileName,"SavedFiles/robot%d.dat",fileIndex);
	while ( File_Exists(fileName) ) {
		fileIndex++;
		sprintf(fileName,"SavedFiles/robot%d.dat",fileIndex);
	}

	return( fileIndex );
}

void ROBOT::Group_Joints(void) {

	// assign the same groupID to all marked joints, then unmark 

	// TBD: Group_Joints() only works for joints not already in a group

	int i;
	int maxID = -1;

	// find the highest current groupID
	for ( i=0; i < numJoints; i++ ) {
		if ( joints[i]->groupID > maxID )
			maxID = joints[i]->groupID;
	}

	maxID++;

	for ( i=0; i < numJoints; i++ ) {
		if ( joints[i]->markIndex > -1 ) {
			joints[i]->groupID = maxID;
			joints[i]->Unmark();
		}
	}
}

void ROBOT::Initialize(ifstream *inFile) {

	(*inFile) >> numObjects;

	objects = new OBJECT * [numObjects];

	for (int i=0;	i<numObjects;	i++)

		objects[i] = new OBJECT(this,inFile);

	(*inFile) >> numJoints;

	joints = new JOINT * [numJoints];

	for (int j=0;	j<numJoints;	j++)

		joints[j] = new JOINT(this,inFile);

	(*inFile) >> botType;

	hidden = false;
	physicalized = false;

	controller = NULL;

	sensorDifferences = 0.0;
}

void ROBOT::Initialize(ROBOT *other) {

	numObjects = other->numObjects;

	objects = new OBJECT * [numObjects];

	for (int i=0;	i<numObjects;	i++)
		objects[i] = new OBJECT(this,other->objects[i]);

	numJoints = other->numJoints;

	joints = new JOINT * [numJoints];

	for (int j=0;	j<numJoints;	j++) {
		joints[j] = new JOINT(this,other->joints[j]);
		joints[j]->hidden = true;
	}

	botType = other->botType; 
	hidden = false;
	physicalized = false;

	if ( other->controller )
		controller = new CONTROLLER(other->controller);
	else
		controller = NULL;

	sensorDifferences = 0.0;
}

void ROBOT::Joint_Delete(int jointIndex) {

	if ( numJoints < 1 || jointIndex < 0 || jointIndex >= numJoints )
	        return;

	// allow deletion of the last joint (user might want no joints)
	delete joints[jointIndex];

	joints[jointIndex] = NULL;

	if ( jointIndex < numJoints - 1 ) {
		// not the last joint; re-index:
		for (int i=jointIndex+1; i<numJoints; i++) {

			joints[i-1] = joints[i];
		}
	}

	numJoints--;
}

void ROBOT::Controller_Set_Sensors(void) {

	int    sensorIndex = 0;
	double sensorValue = 0.0;

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] ) {

			if ( objects[i]->lightSensor ) {

				sensorValue = objects[i]->lightSensor->Get_Value();
				
				controller->Sensor_Set(sensorIndex,sensorValue);

				sensorIndex++;
			}

			if ( objects[i]->touchSensor ) {

				sensorValue = objects[i]->touchSensor->Get_Value();
				//sensorValue = 0.0;

				controller->Sensor_Set(sensorIndex,sensorValue);

				sensorIndex++;
			}
		}

	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] ) {

			if ( joints[j]->proprioceptiveSensor ) {

				sensorValue = joints[j]->proprioceptiveSensor->Get_Value();
				//sensorValue = 0.0;

				controller->Sensor_Set(sensorIndex,sensorValue);

				sensorIndex++;
			}
		}
}

void ROBOT::Controller_Update(int timeStep) {

	if ( !controller )
		return;

	controller->Update(timeStep);
}

void   ROBOT::Sensors_Touch_Print(void) {

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )

			objects[i]->Sensor_Touch_Print();

	printf("\n");
}

double ROBOT::Sensors_Get_Largest_Difference(ROBOT *other) {

	double largestDifferenceInJoints =

		Sensors_In_Joints_Largest_Difference(other);

	double largestDifferenceInObjects =

		Sensors_In_Objects_Largest_Difference(other);

	if ( largestDifferenceInJoints > largestDifferenceInObjects )

		return( largestDifferenceInJoints );

	else
		return( largestDifferenceInObjects );
}

double ROBOT::Sensors_Get_Total_Differences(ROBOT *other) {

//	return(	Sensors_In_Joints_Total_Differences(other) +
//		Sensors_In_Objects_Total_Differences(other) );

	return( Sensors_In_Objects_Total_Differences(other) );
}

double ROBOT::Sensors_Get_Total(void) {

	double sum = 0.0;

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )

			if ( objects[i]->lightSensor ) {
				
				sum = sum + 
				objects[i]->lightSensor->Get_Value();

			}

	return( sum );

}

double ROBOT::Sensors_In_Joints_Largest_Difference(ROBOT *other) {

	double diff = -1000.0;

	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )

			if ( joints[j]->proprioceptiveSensor ) {

/*
				PROP_SENSOR *otherSensor =
				other->joints[j]->proprioceptiveSensor;

				if ( otherSensor ) {
				
					double currDiff = 
					joints[j]->proprioceptiveSensor->
						Difference(otherSensor);

					if ( currDiff > diff )
						diff = currDiff;

					otherSensor = NULL;
				}
*/
			}

	return( diff );
}

double ROBOT::Sensors_In_Joints_Total_Differences(ROBOT *other) {

	double diff = 0.0;
	double num  = 1.0;

	for (int j=0;	j<numJoints;	j++)

		if (	joints[j] && 
			joints[j]->proprioceptiveSensor &&
			other->joints[j] &&
			other->joints[j]->proprioceptiveSensor ) {
 
			double myVal	= joints[j]->proprioceptiveSensor->Get_Value();
			double otherVal = other->joints[j]->proprioceptiveSensor->Get_Value();;


			diff = diff + fabs(myVal - otherVal);
			num++;
		}

	return( diff/num );
}

void ROBOT::Sensors_In_Joints_Write(void) {

	for (int j=0;	j<numJoints;	j++)

		if ( joints[j] )

			if ( joints[j]->proprioceptiveSensor )

				joints[j]->proprioceptiveSensor->Write();
}

double ROBOT::Sensors_In_Objects_Largest_Difference(ROBOT *other) {

	double diff = -1000.0;

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] ) {

			if ( objects[i]->lightSensor ) {

				LIGHT_SENSOR *otherSensor =
				other->objects[i]->lightSensor;

				if ( otherSensor ) {
				
					double currDiff = 
						objects[i]->lightSensor->
						Difference(otherSensor);

					if ( currDiff > diff )
						diff = currDiff;

					otherSensor = NULL;
				}
			}

/*
			if ( objects[i]->touchSensor ) {

				TOUCH_SENSOR *otherSensor =
				other->objects[i]->touchSensor;

				if ( otherSensor ) {
				
					double currDiff = 
						objects[i]->touchSensor->
						Difference(otherSensor);

					if ( currDiff > diff )
						diff = currDiff;

					otherSensor = NULL;
				}
			}
*/
		}

	return( diff );
}

double ROBOT::Sensors_In_Objects_Total_Differences(ROBOT *other) {

	double diff = 0.0;
	double num  = 1.0;

	// assumes that every object has a light sensor
	int objCount = numObjects > other->numObjects ? 
			other->numObjects : 
			numObjects;

	for (int i=0;	i<objCount;	i++)

		if (	objects[i] && 
			objects[i]->lightSensor &&
			other->objects[i] &&
			other->objects[i]->lightSensor ) {

			double myVal	= objects[i]->lightSensor->Get_Value();
			double otherVal = other->objects[i]->lightSensor->Get_Value();

			diff = diff + fabs(myVal - otherVal);
			num++;
		}

	return( diff/num );
}

void ROBOT::Sensors_In_Objects_Write(void) {

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )

			if ( objects[i]->lightSensor )

				objects[i]->lightSensor->Write();

}

void ROBOT::Sensors_Touch_Clear(void) {

	for (int i=0;	i<numObjects;	i++)

		if ( objects[i] )

			objects[i]->Sensor_Touch_Clear();
}

#endif
