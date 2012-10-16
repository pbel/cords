// An open-loop controller defines the outgoing motor commands.

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "matrix.h"

class CONTROLLER {

public:

	MATRIX  *motors;
	MATRIX  *sensorValues;
	MATRIX  *temp;
	int	numSensors;
	int	numTimeSteps;
	int     numMotorGroups;
	int	inAFixedAttractor;

private:
	int	ID;
	double  fitness;
	int	evaluated;
	int	genomesEvaluatedWhenCreated;

public:
	CONTROLLER( int myID, int nS, int nTS, int nM , int evals );
	CONTROLLER( CONTROLLER *other );
	CONTROLLER(ifstream *inFile);
	~CONTROLLER();
	int    Age_Get(int evalsCurrent);
	int    Beats(CONTROLLER *other);
	double Density(void);
	int    Evaluated(void);
	int    Evals_Get(void);
	int    Fitness_Equal_To(double fit);
	double Fitness_Get(void);
	void   Fitness_Set(double fit);
	int    Fitness_Worse_Than(double fit);
	double Get_Motor_Neuron_Value(int motorIndex,int timeSeqNum);
	int    ID_Get(void);
	void   ID_Set(int myID);
	void   Load_From_File(char *fileName);
	void   Motors_Set_Undulate(void);
	void   Motors_Set_Walk(void);
	void   Mutate(double mutationProbability);
	void   Print(void);
	void   Reset(void);
	void   Save(ofstream *outFile);
	void   Sensor_Set(int sensorIndex, double sensorValue);
	void   Update(int timeStep);

private:
//	void    Connection_Add(int nodeIndex);
//	void    Connection_Remove(int nodeIndex);
//	void    EdgeList_Create(void);
	void     Get_File_Dimensions(char *fileName, int *lines, int *chars);
	int	FlipCoin(void);
//	double	Incoming_Connections(int nodeIndex);
	void	Initialize(void);
	void	Mutate(int i, int j);
	int	Mutate_Biased(double mutationProbability);
	double  Rand(double min, double max);
	int 	RandInt(int min, int max);
	double  Scale(double value, double min1, double max1,
                                    double min2, double max2);
	void	Weights_Initialize_NumIncomingConnections(int nic);
};

#endif
