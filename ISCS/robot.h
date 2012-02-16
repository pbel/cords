/* A robot is composed of

	one or more objects,
	one or more joints,
	one or more sensors,
	one or more motors, and
	a neural network controller. */

#ifndef _ROBOT_H
#define _ROBOT_H

#include "object.h"
#include "joint.h"

#include "environment.h"
#include "neuralNetwork.h"

class ROBOT {

public:
	class ENVIRONMENT *containerEnvironment;
	int 		  numObjects;
	int		  activeComponent;// index into components
	OBJECT 		  **objects;
	int		  numJoints; // objects and joints are components
	JOINT		  **joints;
	int 		  hidden;
	int 		  physicalized;
	NEURAL_NETWORK	  *neuralNetwork;
	double		  sensorDifferences;

public:
	ROBOT(ENVIRONMENT *cE, int robotType);
	ROBOT(ROBOT *other);
	ROBOT(ENVIRONMENT *cE, ROBOT *other);
	ROBOT(ENVIRONMENT *cE, ifstream *inFile);
	~ROBOT();
	void Activate(void);
	void Activate_Component(void);
	void Active_Component_Copy(void);
	void Active_Component_Delete(void);
	void Active_Component_Decrement(void);
	void Active_Component_Increment(void);
	void Active_Component_Move(double,double,double);
	void Active_Component_Resize(double,double,double);
	void Active_Component_Rotate(double,double,double);
	void Active_Joint_Range_Change(double);
	int  Components_Number_Of(void);
	void Connect_Robot_Joint(void);
	void Deactivate(void);
	void Deactivate_Component(void);
	void Draw(void);
	double Fitness_Get(ROBOT *targetRobot);
	int  Has_Stopped(void);
	void Hide(void);
	void Hide_Robot_Joints(void);
	int  In_Simulator(void);
	int  Joint_Is_Unattached(void);
	void Label(class NEURAL_NETWORK *genome, int environmentIndex);
	void Make_Incorporeal(void);
	void Make_Physical(dWorldID world, dSpaceID space);
	void Mark_Component(void);
	int  Motors_Number_Of(void);
	void Move(int timeStep);
	void Move_WH(int timeStep);
	void Move(double x, double y, double z);
	void Save(ofstream *outFile);
	double Sensor_Sum(void);
	void Sensors_Add_Difference(ROBOT *other);
	int  Sensors_Number_Of(void);
	void Sensors_Update(void);
	void Sensors_Write(void);
	void Set_Color(double r, double g, double b);
	void Unattached_Joints_Unhide(void);
	void Unhide(void);
	void Unmark_All(void);
	void Unmark_Component(void);

private:
	void Actuate_Motors(void);
	void Actuate_Motors_WH(int);
	void Component_Reindex_After_Deletion(void);
	void Connect_Joint(void);
	void Create_Sandbox(void);
	void Create_Sandbox_Joints(void);
	void Create_Sandbox_Objects(void);
	void Create_Starfish(void);
	void Create_Starfish_Joints(void);
	void Create_Starfish_Objects(void);
	bool File_Exists(char *fileName);
	int  File_Index_Next_Available(void);
	void Initialize(ROBOT *other);
	void Initialize(ifstream *inFile);
	void Joint_Delete(int jointIndex);
	void Neural_Network_Set_Sensors(void);
	void Neural_Network_Update(int timeStep);
	void   Sensors_Touch_Print(void);
	double Sensors_Get_Largest_Difference(ROBOT *other);
	double Sensors_Get_Total_Differences(ROBOT *other);
	double Sensors_Get_Total(void);
	double Sensors_In_Joints_Largest_Difference(ROBOT *other);
	double Sensors_In_Joints_Total_Differences(ROBOT *other);
	void   Sensors_In_Joints_Write(void);
	double Sensors_In_Objects_Largest_Difference(ROBOT *other);
	double Sensors_In_Objects_Total_Differences(ROBOT *other);
	void   Sensors_In_Objects_Write(void);
	void   Sensors_Touch_Clear(void);
};

#endif
