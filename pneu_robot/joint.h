/* This class defines a joint, which is a structure that combines two objects together and allows them to rotate around one another. */

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#ifndef _JOINT_H
#define _JOINT_H

#include "propSensor.h"

class JOINT {

public:
	class   ROBOT 	*containerRobot;
	PROP_SENSOR 	*proprioceptiveSensor;
	int		active; // for selection by user
	int		markIndex;
	int		groupID; // for ganged actuation
	int		hidden;
	double  	axisX, axisY, axisZ;
	double		color[3];
	// The joint's limits, expressed in degrees.
	double		maxFlexion, maxExtension;
	int		obj1Index;
	int		obj2Index;
	double		x,y,z;
	double		angle;
	int		jointType;

private:
	double		pos[3];
	dReal		R[12];
	dJointID 	joint;
	int		physicalized;

public:
	JOINT(void);
	JOINT(ROBOT *cR, int obj1Index, int obj2Index,
	      int jointType,
	      double posX, double posY, double posZ,
	      double axX,  double axY,  double axZ,
	      double maxF, double maxE);
	JOINT(ROBOT *cR, int obj1Index, int obj2Index,
	      int jointType,
	      double posX, double posY, double posZ,
	      double axX,  double axY,  double axZ,
	      double maxF, double maxE, int groupID);
	JOINT(ROBOT *cR, JOINT *other);
	JOINT(ROBOT *cR, ifstream *inFile);
	~JOINT(void);
	void Activate(void);
	void Connect(void);
	void Deactivate(void);
	void Disconnect(void);
	void Draw(void);
	void Hide(void);
	void Initialize_Unattached(void);
	int Is_Unconnected(void);
	void Make_Incorporeal(void);
	void Make_Physical(dWorldID world);
	void Mark(void);
	void Move(double motorNeuronValue);
	void Move(int robotType, double motorNeuronValue);
	void Move(double deltaX, double deltaY, double deltaZ);
	void Rotate(double angleX, double angleY, double angleZ);
	void Save(ofstream *outFile);
	void Sensor_Proprioceptive_Add(void);
	int  Sensors_Number_Of(void);
	void Sensors_Update(void);
	void Set_Color(double r, double g, double b);
	void Unhide(void);
	void Unmark(void);

private:
	double Degrees_To_Radians(double degrees);
	double Radians_To_Degrees(double radians);
	void Draw_Geometrically(void);
	void Initialize(int o1Index, int o2Index,
			int jType,
			double posX, double posY, double posZ,
			double axX,  double axY,  double axZ,
			double maxF, double maxE);
	void Mark_Joint(void);
	double Rand(double min, double max);
	void Remove_From_Simulator(void);
        double Scale(double value, double min1, double max1,
                         double min2, double max2);
	void Sensors_Reset(void);
};

#endif
