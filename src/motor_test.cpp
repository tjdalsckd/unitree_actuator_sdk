#include "serialPort/SerialPort.h"
#include <csignal>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <functional>
#include <cstdio>
#include <atomic>

using namespace std;
using namespace Eigen;
using namespace chrono;

class Timer {
public:
    ~Timer() {
        if (mRunning) {
            stop();
        }
    }
    typedef std::chrono::microseconds Interval;
    typedef std::function<void(void)> Timeout;

    void start(const Interval &interval, const Timeout &timeout) {
        mRunning = true;

        mThread = std::thread([this, interval, timeout] {
            while (mRunning) {
                std::this_thread::sleep_for(interval);

                timeout();
            }
        });
    }
    void stop() {
        mRunning = false;
        mThread.join();
    }


private:
    std::thread mThread{};
    std::atomic_bool mRunning{};
};


class Motor{
	VectorXd q;
	VectorXd dq;
	VectorXd prev_q;
	VectorXd init_q;
	VectorXd torques;
	VectorXd measured_torques;
	double sampling_time;
	double dt;
	//Timer tm;


	SerialPort *serial;
public:
    int dof;
    vector<MOTOR_send> motor_send_list;
    vector<MOTOR_recv> motor_recv_list;    
        Motor(){
           
        }
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
                this->motor_send_list.at(i).T = this->torques[i];
                modify_data(&this->motor_send_list.at(i));

			}
		}
		void send(){
            /*
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
            */
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
double t = 0;

double sampling_time = 1000;
double dt = sampling_time/1000.0/1000.0;

Motor motor;
SerialPort serial("/dev/ttyUSB0",78,4800000,sampling_time);
void* sendData(){
    VectorXd torques;    
    VectorXd q;
    VectorXd dq;    
    q.resize(motor.dof);
    dq.resize(motor.dof);
    torques.resize(motor.dof);    
    motor.getJointStates(q,dq);
    torques[0] = 0.1*sin(2*3.141592*t);
    motor.setTorques(torques);    
    for(int i = 0;i<motor.dof;i++){
                serial.sendRecv(&motor.motor_send_list.at(i),&motor.motor_recv_list.at(i));
                extract_data(&motor.motor_recv_list.at(i));
    }    
    t=t+dt;
}
int main(int argc, char** argv){
    Timer tm;

    // set the serial port name

    int robot_dof = 1;
    motor=Motor(robot_dof,&serial,sampling_time);
    tm.start(std::chrono::microseconds(int(sampling_time)), sendData);    
    VectorXd q;
    VectorXd dq;
    VectorXd torques;
    q.resize(robot_dof);
    dq.resize(robot_dof);
    torques.resize(robot_dof);
    double endTime = 5.0;
   	chrono::system_clock::time_point StartTime = chrono::system_clock::now();
    while(t<endTime){
        usleep(1);
    }
    chrono::system_clock::time_point EndTime = chrono::system_clock::now();
   chrono::microseconds micro = chrono::duration_cast<chrono::microseconds>(EndTime - StartTime);
    cout << "chrono time : "<<micro.count()/1000.0/1000.0 << "sec" << endl;


    cout << t << "--sec" << endl;
    torques[0] = 0;
    motor.setTorques(torques);
	//motor.send();
    return 0;
}
