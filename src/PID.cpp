#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	i_error_ = 0;
  	d_error_ = 0;
  	p_error_ = 0;
  	init_p_error_ = false;
}

void PID::UpdateError(double cte) {
    //PID_error = (-tau_proportional*cte)-(tau_derivative*(cte-cte_previous))-(tau_integral*cte_sum);
    // update the error
    if(!init_p_error_)
    {
    	p_error_ = cte;
    	init_p_error_ = true;
    }

    i_error_ += cte;
  	d_error_ = (cte - p_error_);
  	p_error_ = cte;
}

double PID::TotalError() {
	//PID_error = (-Kp*cte)-(Kd*(cte-cte_previous))-(Ki*cte_sum);
	return (-(Kp_*p_error_)-(Kd_*d_error_)-(Ki_*i_error_));
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
	std::string reset_msg = "42[\"reset\",{}]";
	ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
