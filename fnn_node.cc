/*
 * FNN_Node.cpp
 *
 *  Created on: 20 Nov, 2013
 *      Author: along
 */

#include "fnn_node.h"
#include <cmath>
#include <iostream>

using namespace std;

const int NUMOFPOINTS = 100; //used to calculate center of area

FNN_Node::FNN_Node()
{
	center = right_spread = left_spread = 0;
	weightedInput = 0;
}

void FNN_Node::SetupFNNNode(bool adjustable = false, int layer = 1, double min = 0, double max = 0)
{
	this->adjustable = adjustable;
	this->layer = layer;
	this->min = min;
	this->max = max;
}

void FNN_Node::GetParameters(double &left, double &center, double &right)
{
	left = this->left_spread;
	center = this->center;
	right = this->right_spread;
}

void FNN_Node::SetParameters(double left, double center, double right)
{
	if(adjustable)
	{
		cout<<"["<<this->left_spread<<" "<<this->center<<" "<<this->right_spread<<"] --> ";
		this->center = center;
		this->right_spread = right;
		this->left_spread = left;
		cout<<"["<<this->left_spread<<" "<<this->center<<" "<<this->right_spread<<"]"<<endl;
	}
	else
		cout<<"FNN_Node is not adjustable, no parameter is needed!"<<endl;
}

double FNN_Node::MF(double x)		//triangular function
{
//	cout<<"center: "<<center<<"  left: "<<center-left_spread<<" right: "<<center+right_spread<<endl;
//	cout<<"input: "<<x<<" ";
	if(x >= center && x <= center + right_spread)
		return 1 - fabs(x-center) / right_spread;
	else if( x >= center - left_spread && x < center)
		return 1 - fabs(x-center) / left_spread;
	else
		return 0;
}

double FNN_Node::Calculate_COA()
{
	double step = (max - min)/(NUMOFPOINTS - 1);
	double total_mf = 0, weighted_sum = 0;

	for(int i = 0; i<NUMOFPOINTS; i++)
	{
		double x = min + step*i;
		total_mf += MF(x);
		weighted_sum += MF(x) * x;
	}
	return weighted_sum / total_mf;
}

void FNN_Node::IntegrationFunction(vector<double> input, vector<double> input_bak) //input_bak only used in layer 5, representing the output of layer 3
{
	switch(layer)
	{
		case 1:
		{
			weightedInput = input[0];
			break;
		}
		case 2:
		{
			weightedInput = input[0];
			break;
		}

		case 3:								//perform "softmin" operation to match preconditions
		{
			double numerator = 0, dominator = 0;
			for(int i = 0; i<input.size(); i++)
			{
				numerator += input[i] * exp(-K*input[i]);
				dominator += exp(-K*input[i]);
			}
			weightedInput = numerator / dominator;
			break;
		}
		case 4:								//sum up fire strengths
		{
			double sum_of_input = 0, sum_of_input_squre = 0;
			for(int i = 0; i<input.size(); i++)
			{
					sum_of_input += input[i];
					sum_of_input_squre += input[i] * input[i];
			}
			weightedInput = (center + 0.5*(right_spread - left_spread)) * sum_of_input - 0.5*(right_spread - left_spread) * sum_of_input_squre;
			break;
		}
		case 5:								//calculate weighted sum
		{
			double sum_layer4_output = 0, sum_layer3_output = 0;
			for(int i = 0; i<input.size(); i++)
				sum_layer4_output += input[i];
			for(int i = 0; i<input_bak.size(); i++)
				sum_layer3_output += input_bak[i];

			weightedInput = sum_layer4_output / sum_layer3_output;
			break;
		}
	}

}

double FNN_Node::ActivateFunction()
{
	switch(layer)
	{
	case 1:						//a = f
		return weightedInput;

	case 2:
		if (weightedInput > max)			//check domain
		{
			cout<<"weightedInput overflow! weightedInput:"<<weightedInput<<" max: "<<max<<endl;
			weightedInput = max;
		}
		else if(weightedInput < min)
		{
			cout<<"weightedInput overflow! weightedInput:"<<weightedInput<<" min: "<<min<<endl;
			weightedInput = min;
		}
		return MF(weightedInput);	//bell shaped function

	case 3:
		//return weightedInput < 1 ? weightedInput : 1;		//min operation
		return weightedInput;
	case 4:
		return weightedInput;

	case 5:													//a = f
		return weightedInput;
	}
}

double FNN_Node::LMOM(double wr)
{
	return center + 0.5 * (right_spread - left_spread) * (1 - wr);
}

double FNN_Node::Derivative_LMOM(double wr)
{
	return -0.5*(right_spread - left_spread);
}

double FNN_Node::GetInputValue()
{
	return weightedInput;
}

