/*
 * aen.h
 *
 *  Created on: 2013Äê12ÔÂ25ÈÕ
 *      Author: along
 */

#ifndef AEN_H_
#define AEN_H_

#include <vector>
using namespace std;

class AEN
{
private:
	int num_of_layers;
	int number_input;
	double *xt, *xt1, *y_t_t, *y_t_t1;		//node output
	double **a, *b, *c;		//weight
	int t;					//timestamp
	double v_t_t, v_t_t1;	//prediction of reinforcement
	double r;				//internal reinforcement
	double discount_rate;
	double beta, beta_h;	//constants that used to update weight

	int sgn(double x)
	{
		return x >= 0 ? 1:-1;
	}
public:
	AEN(int number_input);
	void Initialize();
	double Evaluate(vector<double> input, double external_reinforcement);
	void UpdateWeight();
};



#endif /* AEN_H_ */
