/*
 * Movingavg.h
 *
 *  Created on: March 28, 2015
 *      Author: Kandithws
 */

#ifndef MOVINGAVG_H
#define MOVINGAVG_H

#define DEFAULT_WINDOW_SIZE 50

#include <new>


class Movingavg
{
	private:
		double *buff;
		unsigned int iter;
 		double norm;
		unsigned int winsize;
		bool isfull;
		bool push_buff(double data);

 	public:
 		Movingavg();
 		Movingavg(unsigned int windowsize);
 		~Movingavg();

 		bool compute(double data,double &output);
 		void clearbuff();

};

#endif