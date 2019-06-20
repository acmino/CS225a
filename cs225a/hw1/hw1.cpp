#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4
#define QUESTION_5   5

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_controller.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

    ofstream ofs; 
    ofs.open("q1.txt");

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd g(dof);
    VectorXd c(dof); 
    cout << "mass " << robot->_M << "\n";

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 54.0;      // chose your d gain

			VectorXd q_desired(7);
            q_desired << M_PI/2,-M_PI/4,0,-2.1816,0,1.396,0;   // change to the desired robot joint angles
           
			command_torques = -kp*(robot->_q - q_desired) - kv*robot->_dq;  // change to the control torques you compute
            ofs << robot->_q(0)<<","<< robot->_q(2)<<","<< robot->_q(3)<<"\n";
           
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
            double kp = 400.0;      // chose your p gain
			double kv = 50.0;      // chose your d gain

			VectorXd q_desired(7);
            q_desired << M_PI/2,-M_PI/4,0,-2.1816,0,1.396,0;   // change to the desired robot joint angles
            robot->gravityVector(g); 
			command_torques = -kp*(robot->_q - q_desired) - kv*robot->_dq + g; 
            ofs << robot->_q(0)<<","<< robot->_q(2)<<","<< robot->_q(3)<<"\n";
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
            double kp = 400.0;      // chose your p gain
			double kv = 40.0;      // chose your d gain

			VectorXd q_desired(7);
            q_desired << M_PI/2,-M_PI/4,0,-2.1816,0,1.396,0;   // change to the desired robot joint angles
            robot->gravityVector(g); 
			command_torques = (robot->_M)*(-kp*(robot->_q - q_desired) - kv*robot->_dq) + g; 
            ofs << robot->_q(0)<<","<< robot->_q(2)<<","<< robot->_q(3)<<"\n";

		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
            double kp = 400.0;      // chose your p gain
			double kv = 40.0;      // chose your d gain

			VectorXd q_desired(7);
            q_desired << M_PI/2,-M_PI/4,0,-2.1816,0,1.396,0;   // change to the desired robot joint angles
            robot->gravityVector(g); 
            robot->coriolisForce(c);  
			command_torques = (robot->_M)*(-kp*(robot->_q - q_desired) - kv*robot->_dq) + c + g; 
            ofs << robot->_q(0)<<","<< robot->_q(2)<<","<< robot->_q(3)<<"\n";
		}

		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_5)                                          
		{
            double kp = 400.0;      // chose your p gain
			double kv = 40.0;      // chose your d gain

			VectorXd q_desired(7);
            q_desired << M_PI/2,-M_PI/4,0,-2.1816,0,1.396,0;   // change to the desired robot joint angles
            robot->gravityVector(g); 
            robot->coriolisForce(c);  
			command_torques = (robot->_M)*(-kp*(robot->_q - q_desired) - kv*robot->_dq) + c + g; 
            ofs << robot->_q(0)<<","<< robot->_q(2)<<","<< robot->_q(3)<<"\n";
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    ofs.close();
	return 0;
}
