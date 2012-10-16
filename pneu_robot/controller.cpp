#include "stdio.h"
#include "stdlib.h"
#include <algorithm>

#ifndef _CONTROLLER_CPP
#define _CONTROLLER_CPP

#include "controller.h"
#include "matrix.h"

extern int	CNTRLR_MOTOR_INIT;

CONTROLLER::CONTROLLER(	int myID, int nS, int nTS, int nMG, int evals ) {

	ID		= myID;
	fitness		= 0.0;
	evaluated	= false;

	numSensors	= nS;

	numTimeSteps	= nTS;

	numMotorGroups 	= nMG;	// optimizer references as "numMotors"

	genomesEvaluatedWhenCreated = evals;

//	printf("controller: numTimeSteps: %d, numMotorGroups: %d\n",
//	       numTimeSteps, numMotorGroups);

	// motors matrix contains only motor values 0 or 1
	// initialize to ones
	// TBD: these are really motor groups
	motors	= new MATRIX(numTimeSteps,numMotorGroups,1);

	Initialize();

//	motors->Print();
//	exit(0);
}

CONTROLLER::CONTROLLER(	CONTROLLER *other ) {

	motors	 	= new MATRIX(other->motors);

	numSensors	= other->numSensors;

	numTimeSteps	= other->numTimeSteps;

	numMotorGroups	= other->numMotorGroups;

	genomesEvaluatedWhenCreated = other->genomesEvaluatedWhenCreated;

	Initialize();
}

CONTROLLER::CONTROLLER(ifstream *inFile) {

	int isMotors;
	(*inFile) >> isMotors;
	if ( isMotors ) {
		motors = new MATRIX(inFile);
	}
	else
		motors = NULL;

	(*inFile) >> numSensors;

	(*inFile) >> numTimeSteps;

	(*inFile) >> numMotorGroups;

	(*inFile) >> ID;
	(*inFile) >> fitness;
	(*inFile) >> evaluated;
	(*inFile) >> genomesEvaluatedWhenCreated;

	Initialize();
}

CONTROLLER::~CONTROLLER(void) {

	if ( motors ) {
		delete motors;
		motors = NULL;
	}

	if ( temp ) {
		delete temp;
		temp = NULL;
	}

	if ( sensorValues ) {
		delete sensorValues;
		sensorValues = NULL;
	}
}

int    CONTROLLER::Beats(CONTROLLER *other) {

	return( Fitness_Get() > other->Fitness_Get() );
}

double CONTROLLER::Density(void) {

	double density = 0.0;

	for (int j=0;	j<numTimeSteps;	j++)

		for (int i=0;	i<(numMotorGroups);	i++)

			if ( motors->Get(i,j)!=0 )

				density++;

	density = density / double(numTimeSteps * numMotorGroups);

	return( density );
}

int    CONTROLLER::Evals_Get(void) {

	return( genomesEvaluatedWhenCreated );
}

int    CONTROLLER::Evaluated(void) {

	return( evaluated );
}

int    CONTROLLER::Fitness_Equal_To(double fit) {

	return( fitness == fit );
}

double CONTROLLER::Fitness_Get(void) {

	return( fitness );
}

void   CONTROLLER::Fitness_Set(double fit) {

	fitness = fit;

	evaluated = true;
}

int    CONTROLLER::Fitness_Worse_Than(double fit) {

	return( fitness < fit );
}

double CONTROLLER::Get_Motor_Neuron_Value(int motorIndex, int timeSeqNum) {

	int col = motorIndex;

	int row = timeSeqNum;

	return motors->Get(row, col);
}

int  CONTROLLER::ID_Get(void) {

	return( ID );
}

void CONTROLLER::ID_Set(int myID) {

	ID = myID;
}

void CONTROLLER::Load_From_File(char *fileName) {

	ifstream inFile(fileName);

	string line;

//	int numLines = 0;
//	int numChars = 0; // unused; numMotorGroups is not defined by the control file
//
//	Get_File_Dimensions(fileName, &numLines, &numChars);
//
//	printf("lines in controller file: %d\n",numLines);
//	printf("chars in first line: %d\n",numChars);
//	printf("numMotorGroups: %d\n",numMotorGroups);
//
//	printf("note: setting numTimeSteps from file in controller.cpp\n");
//	numTimeSteps = numLines;
//
//	// always use numMotorGroups when reading the file
//	if ( numChars != numMotorGroups )
//		printf("chars in controller file doesn't match numMotorGroups: %d\n",numChars);

	int value;
	for ( int i=0; i<numTimeSteps; i++ ) {
		for ( int j=0; j<numMotorGroups; j++ ) {
			(inFile) >> value;
			motors->Set(i,j,value);
		}
	}
}

void CONTROLLER::Get_File_Dimensions(char *fileName, int *nLines, int *nChars) {

	// # lines and # chars in first line 
	// TBD: number of chars in first line is not used

	ifstream inFile(fileName);

	string line;

	if ( !inFile.is_open() ) {
		printf("Can't open file to load.\n");
	}
	else {
		while ( getline(inFile, line) ) {
			if ( *nLines == 0 ) { // only check one line
//				printf("*lines: %d, line.length: %d\n",*nLines, (int)line.length());
				int i = 0;
				while ( i < (line.length() ) ) {
					if ( isgraph(line.at(i)) )  {
						*nChars = *nChars + 1;
						i++;
						while ( i < line.length() && isgraph(line.at(i)) )
							i++;
					} else {
						i++;
					}
				}
			}
			*nLines = *nLines + 1;
		}
	}
	inFile.close();
}

void CONTROLLER::Motors_Set_Undulate(void) {
	// fixed pattern, for test

	// WGU undulating pattern

	// time LR	RR	unused	unused	LF	RF	unused	unused	body
	// ------------------------------------------------------------------------- 
	// 800	0	0	0	0	0	0	0	0	0
	// 350	1	1	0	0	0	0	0	0	0
	// 750	1	1	0	0	0	0	0	0	1
	// 750	1	1	0	0	1	1	0	0	1
	// 650	0	0	0	0	1	1	0	0	1
	// 100	0	0	0	0	1	1	0	0	0

	motors->SetAllTo(0);

	// GCD is 50 msec

	// for us, 0th column is body

	int i;

	for (i=16; i<23; i++ ) {
		motors->Set(i,1,1);
		motors->Set(i,2,1);
	}
	for (i=23; i<38; i++ ) {
		motors->Set(i,0,1);
		motors->Set(i,1,1);
		motors->Set(i,2,1);
	}
	for (i=38; i<53; i++ ) {
		motors->Set(i,0,1);
		motors->Set(i,1,1);
		motors->Set(i,2,1);
		motors->Set(i,3,1);
		motors->Set(i,4,1);
	}
	for (i=53; i<68; i++ ) {
		motors->Set(i,0,1);
		motors->Set(i,3,1);
		motors->Set(i,4,1);
	}
	for (i=68; i<70; i++ ) {
		motors->Set(i,3,1);
		motors->Set(i,4,1);
	}

	//motors->Print();
	//printf("motors matrix length: %d, width: %d\n",motors->length,motors->width);

}

void CONTROLLER::Motors_Set_Walk(void) {
	// fixed pattern, for test

	// WGU walking pattern

	// time LF	RF	unused	unused	LR	RR	unused	unused	body
	// ------------------------------------------------------------------------- 
	// 750	0	1	0	0	0	0	0	0	1
	// 750	0	0	0	0	1	0	0	0	1
	// 750	1	0	0	0	0	0	0	0	1
	// 750	0	0	0	0	0	1	0	0	1

	motors->SetAllTo(0);

	// for us, 0th column is body

	// body always flexed in this gait
	motors->Set(0,0,1);
	motors->Set(1,0,1);
	motors->Set(2,0,1);
	motors->Set(3,0,1);

	// legs
	motors->Set(0,3,1); // left rear
	motors->Set(1,2,1); // right front
	motors->Set(2,4,1); // right rear
	motors->Set(3,1,1); // left front

	//motors->Print();
	//printf("motors matrix length: %d, width: %d\n",motors->length,motors->width);

}

void CONTROLLER::Mutate(double mutationProbability) {

/*
	for (int i=0;i<numNodes;i++) {

		for (int j=0;j<numNodes;j++) {

			if ( Rand(0,1) < mutationProbability )

				Mutate(i,j);
		}
	}
*/

	int mutationOccurred = Mutate_Biased(mutationProbability);

	while ( !mutationOccurred )

		mutationOccurred = Mutate_Biased(mutationProbability);
}

void CONTROLLER::Print(void) {

	printf("%4.4f (%d)\t",fitness,ID);

/*
	if ( weights )

		weights->Print();

	printf("numTimeSteps: %d, numMotorGroups: %d, numNodes: %d\n",
	        numTimeSteps, numMotorGroups, numNodes);

	if ( nodeValues )

		nodeValues->Print();
*/

}

void CONTROLLER::Reset(void) {

	fitness = 0.0;
	evaluated = false;
}

void CONTROLLER::Save(ofstream *outFile) {

	if ( motors ) {
		(*outFile) 				<< "1\n";
		motors->Write(outFile);
	}
	else
		(*outFile) 				<< "0\n";

        (*outFile) << numSensors  			<< "\n";

        (*outFile) << numTimeSteps  			<< "\n";

        (*outFile) << numMotorGroups  			<< "\n";

        (*outFile) << ID  				<< "\n";
        (*outFile) << fitness  				<< "\n";
        (*outFile) << evaluated  			<< "\n";
        (*outFile) << genomesEvaluatedWhenCreated	<< "\n";
}

void CONTROLLER::Sensor_Set(int sensorIndex, double sensorValue) {

//	sensorValue = sensorValue*1000.0;
//	sensorValue = int(sensorValue);
//	sensorValue = sensorValue/1000.0;

/*
	// Binarize sensor values
	if ( sensorValue < 0.0 )
		sensorValue = -1;
	else
		sensorValue = +1;
*/

	// Shut off sensors
//	sensorValue = 0;

	sensorValues->Set(0,sensorIndex,sensorValue);
}

void CONTROLLER::Update(int timeStep) {

	// Initialize

	for (int j=0;	j<numTimeSteps*numMotorGroups;	j++)

		temp->vals[j] = 0;

//	// Add influences between nodes
//	for (int i=0;	i<numConnections;	i++) {
// 
//		int sourceNode	= int(edgeList->vals[i*edgeList->width + 0]);
//		int targetNode	= int(edgeList->vals[i*edgeList->width + 1]);
//		double weight	=     edgeList->vals[i*edgeList->width + 2];
//
//		// Source node is a sensor...
//		if ( sourceNode < numTimeSteps ) {
//
//			temp->vals[targetNode] = temp->vals[targetNode] + 
//
//						(sensorValues->vals[sourceNode]*weight);
//		}
//		// Source node is a hidden or motor neuron...
//		else {
//			temp->vals[targetNode] = temp->vals[targetNode] + 
//
//						(nodeValues->vals[sourceNode-numTimeSteps]*weight);
//		}
//	}
//
//	for (int j=0;	j<numNodes;	j++) {
//
//		if ( temp->vals[j]      < 0 )
//
//			temp->vals[j]   = -1;
//
//		else
//			temp->vals[j]   = +1;
//	}
//
//	for (int j=0;	j<numNodes;	j++) {
//
//		// Update the two motor and two hidden nodes,
//		// but don't change the two sensor nodes.
//
//		nodeValues->vals[j] = temp->vals[j];
//	}
}

// Private methods ---------------------------------------------

//void   CONTROLLER::Connection_Add(int nodeIndex) {
//
//	int i = RandInt(0,(numTimeSteps+numNodes)-1);
//
//	while ( weights->Get(i,nodeIndex) != 0 )
//
//		i = RandInt(0,(numTimeSteps+numNodes)-1);
//
//	if ( FlipCoin() )
//
//		weights->Set(i,nodeIndex,-1);
//	else
//		weights->Set(i,nodeIndex,+1);
//}

//void   CONTROLLER::Connection_Remove(int nodeIndex) {
//
//	int i = RandInt(0,(numTimeSteps+numNodes)-1);
//
//	while ( weights->Get(i,nodeIndex) == 0 )
//
//		i = RandInt(0,(numTimeSteps+numNodes)-1);
//
//	weights->Set(i,nodeIndex,0);
//}

//void CONTROLLER::EdgeList_Create(void) {
//
//	for (int i=0;i<weights->length;i++)
//
//		for (int j=0;j<weights->width;j++)
//
//			if ( weights->Get(i,j) != 0.0 )
//
//				numConnections++;
//
//	edgeList = new MATRIX(numConnections,3,0.0);
//
//	numConnections = 0;
//
//	for (int i=0;i<weights->length;i++)
//
//		for (int j=0;j<weights->width;j++)
//
//			if ( weights->Get(i,j) != 0.0 ) {
//
//				edgeList->Set(numConnections,0,i);
//				edgeList->Set(numConnections,1,j);
//				edgeList->Set(numConnections,2,weights->Get(i,j));
//				numConnections++;
//			}
//}

int CONTROLLER::FlipCoin(void) {

        return( Rand(0.0,1.0) > 0.5 );
}

//double CONTROLLER::Incoming_Connections(int nodeIndex) {
//
//	double incomingConnections = 0.0;
//
//	for (int i=0;	i<(numTimeSteps+numNodes);	i++)
//
//		if ( weights->Get(i,nodeIndex) != 0 )
//
//			incomingConnections++;
//
//	return( incomingConnections );
//}

void CONTROLLER::Initialize(void) {

// TBD: use a constant for now to determine how to initialize

	switch ( CNTRLR_MOTOR_INIT ) {
	case 0: // zeros
		motors->SetAllTo(0);
		break;
	case 1: // ones
		motors->SetAllTo(1);
		break;
	case 2: // random ones and zeros
		for ( int i=0; i<numTimeSteps; i++ ) {
			for ( int j=0; j<numMotorGroups; j++ ) {
				if ( motors->Get(i,j) < 0.5 ) 
					motors->Set(i,j,0);
				else
					motors->Set(i,j,1);
			}
		}
		break;
	case 3:
		Motors_Set_Walk();
		break;
	case 4:
		Motors_Set_Undulate();
		break;
	default:
		motors->SetAllTo(1);
	}

	//motors->Print_Plain();
	//printf("\n");

	temp 		= new MATRIX(1,numTimeSteps*numMotorGroups,0.0);

	inAFixedAttractor = false;

	sensorValues	= new MATRIX(1,numSensors,0.0);
}

void CONTROLLER::Mutate(int i, int j) {
	// sets motor values to 0 or 1 only 

	int originalValue = int(motors->Get(i,j));

	switch ( originalValue ) {

	case 0:
		if ( FlipCoin() )
			motors->Set(i,j,1);
		break;

	case 1:
		if ( FlipCoin() )
			motors->Set(i,j,0);
		break;
	}
}

int CONTROLLER::Mutate_Biased(double mutationProbability) {

	int mutationOccurred = false;

	for (int i=0; i<numTimeSteps; i++) {

		for (int j=0; j<numMotorGroups; j++) {

			if ( Rand(0,1) < mutationProbability ) {

				Mutate(i,j);

				mutationOccurred = true;
			}
		}
	}

	motors->Print();
	printf("\n");

	return( mutationOccurred );
}

double CONTROLLER::Rand(double min, double max) {

        double zeroToOne = ((double)rand()) / RAND_MAX;
        double returnVal;

        returnVal = (zeroToOne * (max-min)) + min;
        return returnVal;
}

int CONTROLLER::RandInt(int min, int max) {

        if ( min == max )
                return( min );
        else
                return( (rand() % (max-min+1)) + min );
}

double CONTROLLER::Scale(double value, double min1, double max1,
                                           double min2, double max2) {

        if ( min1 < 0 )
                value = value - min1;
        else
                value = value + min1;

        return( (value*(max2-min2)/(max1-min1)) + min2 );

}

#endif
