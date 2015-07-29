/*
 * Movingavg.cpp
 *
 *  Created on: March 28, 2015
 *      Author: Kandithws
 */

/*
	One-Dimensional Signal Moving Average Filter

*/
#include "Movingavg.h"


Movingavg::Movingavg()
{
	this->iter = 0;

	this->isfull = false;
	this->norm = 1.00/(double)DEFAULT_WINDOW_SIZE;

	this->buff = new double[DEFAULT_WINDOW_SIZE];
}

Movingavg::Movingavg(unsigned int windowsize)
{
	this->iter = 0;

	this->isfull = false;
	this->norm = 1.00/(double)windowsize;

	this->buff = new double[windowsize];
	this->winsize = windowsize;
}

Movingavg::~Movingavg(){}

bool Movingavg::push_buff(double data)
{
	/*Adding data*/
	this->buff[this->iter] = data;
	
	/*Checkif data overrun*/
	if(this->iter == this->winsize-1)
	{
		this->iter = 0;
		return true;

	}
	else 
	{
		this->iter++;
		return false;
	}
		
}



void Movingavg::clearbuff()
{
	for(int i=0 ; i< this->winsize ; i++)
	{
		this->buff[i] = 0;
	}

	this->isfull = false;
}

bool Movingavg::compute(double data,double &output)
{
	/*When buffer is not full -> waiting*/
	if(!this->isfull)
	{
		/*Adding Data to Buffer til full*/
		this->push_buff(data);
		if(this->iter == this->winsize-1 )
		{
			this->isfull = true;
		}	

		return false;
	}
	else
	{
		/*Adding next buff*/
		this->push_buff(data);

		double sum = 0;
		/*Calculating Moving Avg*/
		for(int i=0 ; i< this->winsize ; i++)
		{
			sum += this->buff[i];
		}
		output = this->norm*sum;
		return true;
	}
}

