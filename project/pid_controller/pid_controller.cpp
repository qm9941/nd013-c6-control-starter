/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <limits>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min) {
   // Initialize PID coefficients
   _Ki = Ki;
   _Kp = Kp;
   _Kd = Kd;

   // Initialize PID limits
   _output_lim_max = _output_lim_max;
   _output_lim_min = _output_lim_min;

   // Initialize PID errors
   _Error = 0;
   _Error_previous = 0;
   _Error_integral = 0;
  
   // Init Delta time
   _dt = 0;
}


void PID::UpdateError(double cte) {
   //Store previous error
   _Error_previous = _Error;

   //Store error
   _Error = cte;

   //Update error integral
   _Error_integral += (cte * _dt);
}

double PID::TotalError() {
   // Calculate controller output
   double control;

   //P term
   control = _Error * _Kp;

   //I Term
   control += (_Error_integral * _Ki);

   //D Term
   if (_dt > std::numeric_limits<double>::epsilon())
   {
      control += ((_Error - _Error_previous) / _dt * _Kd);
   }

   //Limit control
   if (control > _output_lim_max)
   {
      control = _output_lim_max;
   }
   else if (control < _output_lim_min)
   {
      control = _output_lim_min;
   }
   
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   // Update the delta time with new value
   _dt = new_delta_time;
}