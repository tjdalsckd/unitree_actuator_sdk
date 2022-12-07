#include "serialPort/SerialPort.h"
#include <csignal>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <iostream>
using namespace std;
using namespace Eigen;
using namespace chrono;
class Motor{
	int dof;
	VectorXd q;
	VectorXd dq;
	VectorXd prev_q;
	VectorXd init_q;
	VectorXd torques;
	VectorXd measured_torques;
	double sampling_time;
	double dt;


	vector<MOTOR_send> motor_send_list;
	vector<MOTOR_recv> motor_recv_list;
	SerialPort *serial;
public:
		Motor(int dof,SerialPort *serial,double sampling_time){
			this->sampling_time = sampling_time; //us
			this->dt = sampling_time/1000.0/1000.0;
			this->serial = serial;
			this->dof = dof;
			this->q.resize(dof);
			this->dq.resize(dof);
			this->prev_q.resize(dof);
			this->init_q.resize(dof);
			this->measured_torques.resize(dof);
			this->torques.resize(dof);
			for(int i = 0;i<dof ;i++){
				this->q[i] =0.0;
				this->prev_q[i] =0.0;
				this->dq[i] =0;
				this->measured_torques[i] = 0.0;
				this->torques[i] = 0.0;

				MOTOR_send motor_send;
				MOTOR_recv motor_recv;
				this->motor_recv_list.push_back(motor_recv);
				this->motor_send_list.push_back(motor_send);
				this->motor_send_list.at(i).id = i;
				this->motor_send_list.at(i).motorType = MotorType::A1;
				this->motor_send_list.at(i).mode = 10;
				this->motor_send_list.at(i).T = 0.0;
				this->motor_send_list.at(i).W = 0.0;
				this->motor_send_list.at(i).Pos = 0.0;
				this->motor_send_list.at(i).K_P = 0.0;
				this->motor_send_list.at(i).K_W = 0.0;

				modify_data(&this->motor_send_list.at(i));
			    serial->sendRecv(&this->motor_send_list.at(i),&this->motor_recv_list.at(i));
      			 extract_data(&this->motor_recv_list.at(i));
      			 init_q[i]  = this->motor_recv_list.at(i).Pos;      			 
			}
			
		};
		void getJointStates(VectorXd &q,VectorXd &dq){
			for(int i = 0;i<this->dof;i++){
				q[i] = this->q[i];
				dq[i] = this->dq[i];
			}
		}
		void setTorques(VectorXd torques){
			for(int i = 0;i<this->dof;i++){
				this->torques[i] = torques[i];	
			}
		}
		void send(){
			for(int i = 0;i<this->dof;i++){
		    	chrono::system_clock::time_point StartTime = chrono::system_clock::now();

				this->motor_send_list.at(i).T = this->torques[i];
				modify_data(&this->motor_send_list.at(i));
				serial->sendRecv(&this->motor_send_list.at(i),&this->motor_recv_list.at(i));
 	     	    extract_data(&this->motor_recv_list.at(i));
				this->q[i] = this->motor_recv_list.at(i).Pos-this->init_q[i];
				this->dq[i] = low_pass_filter(this->dq[i] , (this->prev_q[i] -this->q[i] )/this->dt,1000);
				this->prev_q[i] =this->prev_q[i];
			    chrono::system_clock::time_point EndTime = chrono::system_clock::now();

 	    	    chrono::microseconds micro = chrono::duration_cast<chrono::microseconds>(EndTime - StartTime);
 	    	    if(this->sampling_time> micro.count()-112)
					usleep(int(this->sampling_time-micro.count()-112));
				else{
					cout<<i<<endl;
				}
				
			}
		}
		double low_pass_filter(double prev_value ,double now_value ,double f){
			double w = f/2.0/3.141592;
			double T = this->dt;
			double alpha = T*w/(1.0+T*w);
			double filtered_value = alpha*now_value + (1-alpha)*prev_value;
			return filtered_value;
		}


		virtual ~Motor(){};
};



int main(int argc, char** argv){
    // set the serial port name
    double sampling_time = 3000;
    double dt = sampling_time/1000.0/1000.0;
    SerialPort serial("/dev/ttyUSB0",78,4800000,sampling_time/2.0);
    int robot_dof = 1;
    Motor motor=Motor(robot_dof,&serial,sampling_time);
    VectorXd q;
    VectorXd dq;
    VectorXd torques;
    q.resize(robot_dof);
    dq.resize(robot_dof);
    torques.resize(robot_dof);
    double t = 0;
    double endTime = 5.0;
   	chrono::system_clock::time_point StartTime = chrono::system_clock::now();
    for(double t = 0;t<endTime;t = t+dt){
    	torques[0] = 0.1*sin(2*3.141592*t);
    	motor.setTorques(torques);
 
      	motor.send();

	    motor.getJointStates(q,dq);
	    //cout<<"t :"<<t<<" -- q : "<< q[0]<<endl;
	    //cout<<"t :"<<t<<" -- dq : "<< dq[0]<<endl;
	    //cout<<"t :"<<t<<" -- torques : "<< torques[0]<<endl;
    }
    chrono::system_clock::time_point EndTime = chrono::system_clock::now();
   chrono::microseconds micro = chrono::duration_cast<chrono::microseconds>(EndTime - StartTime);
    cout << micro.count()/1000.0/1000.0 << "sec" << endl;


    cout << t << "--sec" << endl;
    torques[0] = 0;
    motor.setTorques(torques);
	motor.send();
    return 0;
}
