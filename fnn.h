/*
 * fnn.h
 *
 *  Created on: 19 Nov, 2013
 *      Author: along
 */

#ifndef FNN_H_
#define FNN_H_

const double LEARNINGRATE = 0.1;
const double CONVERGENCECONDITION = 0.05;

#include "fnn_node.h"
#include <string>
#include <vector>

using namespace std;

class FNN
{
private:
	int number_of_input;
	int number_of_rules, number_layer2_nodes, number_layer4_nodes;
	FNN_Node *layer1, *layer2, *layer3, *layer4, output_node;
	double *output1, *output2, *output3, *output4, final_output;				//store activation values of nodes in each layer
	double **weight1_2, **weight2_3, **weight3_4, *weight4_5;	//weighted link between two layers
	double learningRate;
	double convergenceCondition;

	double vt, vt_1;					//store prediction of reinforcement, used in learning
	double last_output;

	vector<double> LoadParametersFromFile(string file_name, string para_name, string input_or_output);
	vector<double> GetParametersFromString(string str);
	vector<vector<int> > LoadRulesFromFile(string file_name);

	int sign(double v) {return (v>= 0) ? 1 : -1;}
public:
	FNN(int number_of_input);
	void Initialize(string file_name);
	double Forward(vector<double> input);
	void GetV(double v);
	void Learning(double v, double st_rt = 1); //st_rt is 1 when stochastic action modifier is not used (by default)
};


#endif /* FNN_H_ */
