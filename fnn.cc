/*
 * fnn.cpp
 *
 *  Created on: 19 Nov, 2013
 *      Author: along
 */

#include "fnn.h"
#include <fstream>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <math.h>

FNN::FNN(int number_of_input)
{
	this->number_of_input = number_of_input;
	vt = vt_1 = 0;
	final_output = last_output = 0;
}

vector<double> FNN::GetParametersFromString(string str)
{
	vector<double> paras;
	size_t begin = str.find("[");
	if(begin == string::npos)
		return paras;
	else
	{
		size_t end;
		while( (end = str.find(" ", begin+1))!= string::npos)
		{
			string para = str.substr(begin+1, end-begin-1);
			//cout<<para<<endl;
			paras.push_back(atof(para.c_str()));
			begin = end;
		}
		string para = str.substr(begin+1, str.size()-2-begin);
		paras.push_back(atof(para.c_str()));
		return paras;
	}
}

vector<double> FNN::LoadParametersFromFile(string file_name, string para_name, string input_or_output)
{
	vector<double> parameters;
	fstream fd;
	fd.open(file_name.c_str());
	string line;

	while(getline(fd, line))
	{
		if(line.find(input_or_output) != string::npos)
		{
			int num_search = input_or_output == "Input"? 6 : 8;
			for(int i = 0; i<num_search; i++)
			{
				getline(fd, line);
				if(line.find(para_name) != string::npos)
				{
					vector<double> paras = GetParametersFromString(line);
					for(int i = 0; i<paras.size(); i++)
						parameters.push_back(paras[i]);
				}
			}
		}
	}
	fd.close();
	return parameters;
}

vector<vector<int> > FNN::LoadRulesFromFile(string file_name)
{
	vector<vector<int> > rules;
	fstream fd;
	fd.open(file_name.c_str());
	string line;

	while(getline(fd, line))
	{
		if(line.find("[Rules]") != string::npos)
		{
			while(getline(fd, line))
			{
				vector<int> rule;
				for(int i = 0; i<line.size(); i++)
				{
					if(line[i] == '(')
						break;
					else if(line[i] == ',' || line[i] == ' ')
						continue;
					else
						rule.push_back(line[i]-'0');
				}
				rules.push_back(rule);
			}
		}
	}
	return rules;
}

void FNN::Initialize(string file_name)
{
	//****************************setup nodes in FNN********************************//
	//setup layer1, range of parameters
	vector<double> range = LoadParametersFromFile(file_name, "Range", "Input");
	layer1 = new FNN_Node[number_of_input];
	output1 = new double[number_of_input];
	for(int i = 0, start = 0; i<number_of_input; i++)
	{
		layer1[i].SetupFNNNode(false, 1, range[start], range[start+1]);
		start += 2;
	}

	//setup layer2, parameters of inputs' mf function
	vector<double> mf_parameters = LoadParametersFromFile(file_name, "MF", "Input");
	number_layer2_nodes = mf_parameters.size() / 3;
	layer2 = new FNN_Node[number_layer2_nodes];
	output2 = new double[number_layer2_nodes];
	for(int i = 0, start = 0; i<number_layer2_nodes; i++)
	{
		if(i != 0 && i % 3 == 0)		//each input parameters has three mfs
			start += 2;
		layer2[i].SetupFNNNode(true, 2, range[start], range[start+1]);	//setup layer number and range of linguistic variables
	}
	for(int i = 0, start = 0; i<number_layer2_nodes; i++)				//setup parameters of mfs
	{
		double center = mf_parameters[start+1];
		double left_spread = center - mf_parameters[start];
		double right_spread = mf_parameters[start+2] - center;
		layer2[i].SetParameters(left_spread, center, right_spread);
		start += 3;
	}

	//setup layer3, rules
	vector<vector<int> > fuzzy_rules = LoadRulesFromFile(file_name);
	number_of_rules = fuzzy_rules.size();
	layer3 = new FNN_Node[number_of_rules];
	output3 = new double[number_of_rules];
	for(int i = 0; i<number_of_rules; i++)
		layer3[i].SetupFNNNode(false, 3, 0, 0);

	//setup layer4, consequent node
	vector<double> range_output = LoadParametersFromFile(file_name, "Range", "Output");
	vector<double> mf_parameters_output = LoadParametersFromFile(file_name, "MF", "Output");
	number_layer4_nodes = mf_parameters_output.size() / 3;
	layer4 = new FNN_Node[number_layer4_nodes];
	output4 = new double[number_layer4_nodes];
	for(int i = 0, start = 0; i<number_layer4_nodes; i++)
	{
		layer4[i].SetupFNNNode(true, 4, range_output[0], range_output[1]);
		double center = mf_parameters_output[start+1];
		double left_spread = center - mf_parameters_output[start];
		double right_spread = mf_parameters_output[start+2] - center;
		layer4[i].SetParameters(left_spread, center, right_spread);
		start += 3;
	}
	//setup layer5, only one output node
	output_node.SetupFNNNode(false, 5, 0, 0);

	//*************************setup links(weights) in FNN****************************//
	//weights between layer1 - layer2
	weight1_2 = new double*[number_of_input];
	for(int i = 0; i<number_of_input; i++)
		weight1_2[i] = new double[number_layer2_nodes];

	for(int i = 0; i<number_of_input; i++)
	{
		for(int j = 0; j<number_layer2_nodes; j++)
		{
			if( j >= i*3 && j<(i+1)*3 )
				weight1_2[i][j] = 1;
			else
				weight1_2[i][j] = 0;
		}
	}

	//weights between layer2 - layer3 & weights between layer3 - layer4
	weight2_3 = new double*[number_layer2_nodes];
	for(int i = 0; i<number_layer2_nodes; i++)
		weight2_3[i] = new double[number_of_rules];

	weight3_4 = new double*[number_of_rules];
	for(int i = 0; i<number_of_rules; i++)
		weight3_4[i] = new double[number_layer4_nodes];

	//initialize
	for(int i = 0; i<number_layer2_nodes; i++)
		for(int j = 0; j<number_of_rules; j++)
			weight2_3[i][j] = 0;

	for(int i = 0; i<number_of_rules; i++)
		for(int j = 0; j<number_layer4_nodes; j++)
			weight3_4[i][j] = 0;

	//setup weights according to fuzzy rules
	for(int i = 0; i<fuzzy_rules.size(); i++)
	{
		vector<int> rule = fuzzy_rules[i];
		for(int j = 0; j<rule.size()-1; j++)
		{
			int antecedent = rule[j] + 3*j - 1;
			weight2_3[antecedent][i] = 1;
		}
		int consequent = rule.back() -1;
		weight3_4[i][consequent] = 1;
	}
}

double FNN::Forward(vector<double> input)
{
	//feed input to layer1

	if(input.size() != number_of_input)
	{
		cout<<"input number not match"<<endl;
		return 0;
	}
	else
	{
		for(int i = 0; i<input.size(); i++)
		{
			vector<double> one_input(1, input[i]);
			layer1[i].IntegrationFunction(one_input);
			output1[i] = layer1[i].ActivateFunction();
		}
	}

	//activation in layer2
	for(int i = 0; i<number_layer2_nodes; i++)
	{
		vector<double> layer2_input;
		for(int j = 0; j<number_of_input; j++)
			if(weight1_2[j][i] == 1)
				layer2_input.push_back(output1[j]);
		layer2[i].IntegrationFunction(layer2_input);
		output2[i] = layer2[i].ActivateFunction();
	}

	//activation in layer3
	for(int i = 0; i<number_of_rules; i++)
	{
		vector<double> layer3_input;
		for(int j = 0; j<number_layer2_nodes; j++)
			if(weight2_3[j][i] == 1)
				layer3_input.push_back(output2[j]);
		layer3[i].IntegrationFunction(layer3_input);
		output3[i] = layer3[i].ActivateFunction();
	}

	//activation in layer4
	for(int i = 0; i<number_layer4_nodes; i++)
	{
		vector<double> layer4_input;
		for(int j = 0; j<number_of_rules; j++)
			if(weight3_4[j][i] == 1)
				layer4_input.push_back(output3[j]);
		layer4[i].IntegrationFunction(layer4_input);
		output4[i] = layer4[i].ActivateFunction();
	}

	//final output
	last_output = final_output;
	vector<double> output3_vec, output4_vec;
	for(int i = 0; i<number_of_rules; i++)
		output3_vec.push_back(output3[i]);
	for(int i = 0; i<number_layer4_nodes; i++)
		output4_vec.push_back(output4[i]);

	output_node.IntegrationFunction(output4_vec, output3_vec);
	final_output = output_node.ActivateFunction();

	return final_output;
}

void FNN::GetV(double v)
{
	vt_1 = vt;
	vt = v;
}

void FNN::Learning(double v, double st_rt)	//st_rt is 1 when stochastic action modifier is not used (by default)
{
	int dv_df = sign( v / (final_output - last_output));

	//update consequent labels
	//cout<<endl<<"consequence labels:"<<endl;
	double sum_layer3_output = 0;
	for(int i = 0; i<number_of_rules; i++)
		sum_layer3_output += output3[i];

	for(int i = 0; i<number_layer4_nodes; i++)
	{
		double weight_sum_dz_dc = 0, weight_sum_dz_dr = 0, weight_sum_dz_dl = 0;
		for(int j = 0; j<number_of_rules; j++)
		{
			if(weight3_4[j][i] == 1)
			{
				weight_sum_dz_dc += output3[j];
				weight_sum_dz_dl += output3[j] * (-0.5*(1-output3[j]));
				weight_sum_dz_dr += output3[j] * (0.5*(1-output3[j]));
			}
		}
		double df_dc = weight_sum_dz_dc / sum_layer3_output;
		double df_dr = weight_sum_dz_dr / sum_layer3_output;
		double df_dl = weight_sum_dz_dl / sum_layer3_output;

		double c, r, l;
		layer4[i].GetParameters(l, c, r);
		double delta_c = LEARNINGRATE * st_rt * dv_df * df_dc;
		double delta_r = LEARNINGRATE * st_rt * dv_df * df_dr;
		double delta_l = LEARNINGRATE * st_rt * dv_df * df_dl;

		layer4[i].SetParameters(l+delta_l, c+delta_c, r+delta_r);
	}

	//update antecedent labels
	//cout<<endl<<"antecedent labels:"<<endl;
	/*double df_dw[number_of_rules];
	for(int i = 0; i<number_of_rules; i++)
	{
		double wr = output3[i], z_wr, dz_wr;
		for(int j = 0; j<number_layer4_nodes; j++)
		{
			if(weight3_4[i][j] == 1)
			{
				z_wr = layer4[j].LMOM(wr);
				dz_wr = layer4[j].Derivative_LMOM(wr);
				break;
			}
		}
		df_dw[i] = (z_wr + wr * dz_wr - final_output) / sum_layer3_output;
	}

	double dw_du[number_of_rules][number_layer2_nodes];
	for(int i = 0; i<number_of_rules; i++)
		for(int j = 0; j<number_layer2_nodes; j++)
			dw_du[i][j] = 0;

	for(int r = 0; r<number_of_rules; r++)
	{
		double dominent = 0;
		for(int j = 0; j<number_layer2_nodes; j++)
			if(weight2_3[j][r] == 1)
				dominent += exp(-K*output2[j]);

		for(int j = 0; j<number_layer2_nodes; j++)
		{
			if(weight2_3[j][r] == 1)
				dw_du[r][j] = (exp(-K*output2[j]) * (1+K*(output3[r]-output2[j]))) / dominent;
		}
	}

	double df_du[number_layer2_nodes];
	for(int i = 0; i<number_layer2_nodes; i++)
	{
		df_du[i] = 0;
		for(int j = 0; j<number_of_rules; j++)
		{
			if(weight2_3[i][j] == 1)
				df_du[i] += df_dw[j] * dw_du[j][i];
		}

		//calculating du/dp(du/dc, du/dl, du/dr)
		double center, left, right;
		layer2[i].GetParameters(left, center, right);
		double du_dc = 0, du_dl = 0, du_dr = 0;
		double input = layer2[i].GetInputValue();
		if(input >= center && input < center+right)
		{
			du_dc = fabs(input-center)>=0 ? 1/right : -1/right;
			du_dl = 0;
			du_dr = fabs(input-center);
		}
		else if(input >= center-left && input < center)
		{
			du_dc = fabs(input-center)>=0 ? 1/left : -1/left;
			du_dl = fabs(input-center);
			du_dr = 0;
		}
		else
			du_dc = du_dl = du_dr = 0;

		double df_dc = df_du[i] * du_dc;
		double df_dl = df_du[i] * du_dl;
		double df_dr = df_du[i] * du_dr;

		double delta_center = LEARNINGRATE * st_rt * dv_df * df_dc;
		double delta_left = LEARNINGRATE * st_rt * dv_df * df_dl;
		double delta_right = LEARNINGRATE * st_rt * dv_df * df_dr;

		double l, c, r;
		layer2[i].GetParameters(l, c, r);
		layer2[i].SetParameters(l+delta_left, c+delta_center, r+delta_right);
	}*/
}



