/*
* aen.cpp
*
*  Created on: 2013Äê12ÔÂ25ÈÕ
*      Author: along
*/
#include <cmath>
#include "aen.h"

AEN::AEN(int number_input)
{
	num_of_layers = 3;
	this->num_of_layers = num_of_layers;
	this->number_input = number_input;

	xt = new double[number_input];
	xt1 = new double[number_input];
	y_t_t1 = new double[number_input];
	y_t_t = new double[number_input];

	a = new double*[number_input];
	for(int i = 0; i<number_input; i++)
		a[i] = new double[number_input];
	b = new double[number_input];
	c = new double[number_input];

	discount_rate = 0.5;
	beta = 0.5;
	beta_h = 0.5;
	r = 0;
}

void AEN::Initialize()
{
	t = 0;
	v_t_t = v_t_t1 = 0;
	r = 0;

	//initailized weight
	for(int i = 0; i<number_input; i++)
	{
		b[i] = 1;
		c[i] = 1;
		for(int j = 0; j<number_input; j++)
			a[i][j] = 1;
	}

	for(int i = 0; i<number_input; i++)
		xt[i] = xt1[i] = y_t_t[i] = y_t_t1[i] = 0;
}

double AEN::Evaluate(vector<double> input, double external_reinforcement)
{
	v_t_t = 0;													//v[t, t]
	for(int i = 0; i<number_input; i++)							//y[t, t]
	{
		double sum = 0;
		for(int j = 0; j<number_input; j++)
		{
			sum += a[i][j] * xt[j];
			v_t_t += b[j] * xt[j];
		}
		y_t_t[i] = 1 / (1 + exp(-sum));
		v_t_t += c[i] * y_t_t[i];
	}

	for(int i = 0; i<number_input; i++)							//x[t+1]
	{
		xt[i] = xt1[i];
		xt1[i] = 1 / (1 + exp(-input[i]));
	}

	for(int i = 0; i<number_input; i++)							//y[t, t+1]
	{
		double sum = 0;
		for(int j = 0; j<number_input; j++)
			sum += a[i][j] * xt1[j];
		y_t_t1[i] = 1 / (1 + exp(-sum));
	}

	v_t_t1 = 0;													//v[t, t+1]
	for(int i = 0; i<number_input; i++)
	{
		for(int j = 0; j<number_input; j++)
			v_t_t1 += b[j] * xt1[j];
		v_t_t1 += c[i] * y_t_t1[i];
	}

	r = external_reinforcement + discount_rate * v_t_t1 - v_t_t;
	t++;
	return r;
}

void AEN::UpdateWeight()
{
	for(int i = 0; i<number_input; i++)
		for(int j = 0; j<number_input; j++)
			a[i][j] = a[i][j] + beta_h * r * y_t_t[i] * (1 - y_t_t[i]) * sgn(c[i]) * xt[j];

	for(int i = 0; i<number_input; i++)
	{
		b[i] = b[i] + beta * r * xt[i];
		c[i] = c[i] + beta * r * y_t_t[i];
	}
}



