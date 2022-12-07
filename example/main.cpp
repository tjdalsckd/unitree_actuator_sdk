#include "serialPort/SerialPort.h"
#include <csignal>

int main(int argc, char** argv){
    // set the serial port name
    SerialPort serial("/dev/ttyUSB0",78,4800000,20000);

    // send message struct
    MOTOR_send motor_run, motor_stop;
    // receive message struct
    MOTOR_recv motor_r;
    std::cout<<argv[1]<<std::endl;
    // set the id of motor
    motor_run.id = atoi(argv[1]);
    // set the motor type, A1 or B1
    motor_run.motorType = MotorType::A1;
    motor_run.mode = 10;
    motor_run.T = 0.0;
    motor_run.W = 0.0;
    motor_run.Pos = 0.0;
    motor_run.K_P = 0.1;
    motor_run.K_W = 3.0;

    motor_stop.id = motor_run.id;
    motor_stop.motorType = motor_run.motorType;
    motor_stop.mode = 0;

    motor_r.motorType = motor_run.motorType;

    // encode data into motor commands
    modify_data(&motor_run);
    modify_data(&motor_stop);

    // turn for 3 second
    for(int i(0); i<30; ++i){
        serial.sendRecv(&motor_run, &motor_r);
        // decode data from motor states
        extract_data(&motor_r);
        std::cout << "Pos: " << motor_r.Pos << std::endl;
        usleep(20000);
    }

    // stop the motor
    while(!serial.sendRecv(&motor_stop, &motor_r)){
        usleep(100000);
    }

    return 0;
}
