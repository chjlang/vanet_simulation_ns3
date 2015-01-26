/*
 * node.h
 *
 *  Created on: 20 Nov, 2013
 *      Author: along
 */

#ifndef NODE_H_
#define NODE_H_

#include <vector>
using namespace std;

const int K = 100;

class FNN_Node
{
private:
	bool adjustable;
	double left_spread, center, right_spread;		//parameters of membership function -- triangular function

	double weightedInput;
	int layer;					//indicate which layer the node is in
	double min, max;			//domain of function

	double Calculate_COA();		//calculate center of area
	double MF(double x);		//membership function
public:
	FNN_Node();
	void SetupFNNNode(bool adjustable, int layer, double min, double max);
	void SetParameters(double delta_left, double delta_center, double delta_right);
	void GetParameters(double &left, double &center, double &right);
	void IntegrationFunction(vector<double> input, vector<double> input_bak = vector<double>()); //integrate inputs, weight[i] defaults to 1, only used in layer5
	double ActivateFunction();		//output activated value to the next layer
	double LMOM(double wr);			//return LMOM given weight strength
	double Derivative_LMOM(double wr);		//return derivative LMOM
	double GetInputValue();					//return "weightedinput"
};


#endif /* NODE_H_ */
