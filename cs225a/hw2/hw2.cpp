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

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
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

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

    ofstream ofs;
    ofs.open("h2.txt");
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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);
    VectorXd g(dof);
    VectorXd c(dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);


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
            DiagonalMatrix<double,7> kp(7);
            DiagonalMatrix<double,7> kv(7);
            VectorXd qd(7);
            kp.diagonal() << -400.0, -400.0, -400.0, -400.0, -400.0, -400.0, -50;
            kv.diagonal() << 50, 50, 50, 50, 50, 50, -.3;
            qd = initial_q;
            qd(6) = .1;


            robot->gravityVector(g);
            robot->coriolisForce(c); 
			command_torques = kp*(robot->_q - qd) - kv*robot->_dq + c + g; 

            ofs << robot->_q(6) << "\n";
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
            
            Vector3d xd = Vector3d(0.3,0.1,0.5);
            Vector3d x = Vector3d::Zero();
            int kp = 200;
            int kv = 100;
            int kvj = 50;
            VectorXd F(3);
            
            robot->Jv(Jv, link_name, pos_in_link);
	        robot->taskInertiaMatrix(Lambda, Jv);
            robot->position(x, link_name, pos_in_link);

            F = Lambda*(kp*(xd - x) - kv*(Jv*robot->_dq));
	        robot->nullspaceMatrix(N, Jv);
            robot->gravityVector(g);

			//command_torques = (Jv.transpose())*F + g - kvj*robot->_dq;
            command_torques = (Jv.transpose())*F + g + N.transpose()*robot->_M*(-kvj*robot->_dq);

            ofs << robot->_q(0) <<","<< robot->_q(1)   <<","<< robot->_q(2)   <<","<< robot->_q(3)   <<","<< robot->_q(4)  <<","<<  robot->_q(5)  <<","<<  robot->_q(6) << "\n";
           //ofs << x.transpose() << "\n";
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
            Vector3d xd = Vector3d(0.3,0.1,0.5);
            Vector3d x = Vector3d::Zero();
            int kp = 200;
            int kv = 50;
            int kvj = 50;
            VectorXd F(3);
            
            robot->nullspaceMatrix(N, Jv);
            robot->gravityVector(g);
            robot->Jv(Jv, link_name, pos_in_link);
	        robot->taskInertiaMatrix(Lambda, Jv);
            robot->position(x, link_name, pos_in_link);
        	robot->dynConsistentInverseJacobian(J_bar, Jv);

            F = Lambda*(kp*(xd - x) - kv*(Jv*robot->_dq)) + J_bar.transpose()*g;
			//command_torques = (Jv.transpose())*F + g - kvj*robot->_dq;
            command_torques = (Jv.transpose())*F + g + N.transpose()*robot->_M*(-kvj*robot->_dq);

            //ofs << robot->_q(0) <<","<< robot->_q(1)   <<","<< robot->_q(2)   <<","<< robot->_q(3)   <<","<< robot->_q(4)  <<","<<  robot->_q(5)  <<","<<  robot->_q(6) << "\n";
            ofs << x.transpose() << "\n";
			
        }

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
            Vector3d xd = Vector3d(0.3,0.1,0.5) + .1*Vector3d(sin(M_PI*timer.elapsedTime()), cos(M_PI*timer.elapsedTime()),0);
            Vector3d x = Vector3d::Zero();
            int kp = 200;
            int kv = 50;
            int kvj = 50;
            VectorXd F(3);
            
            robot->nullspaceMatrix(N, Jv);
            robot->gravityVector(g);
            robot->Jv(Jv, link_name, pos_in_link);
	        robot->taskInertiaMatrix(Lambda, Jv);
            robot->position(x, link_name, pos_in_link);
        	robot->dynConsistentInverseJacobian(J_bar, Jv);

            //F = Lambda*(kp*(xd - x) - kv*(Jv*robot->_dq)) + J_bar.transpose()*g;
            F = (kp*(xd - x) - kv*(Jv*robot->_dq)) + J_bar.transpose()*g;

			//command_torques = (Jv.transpose())*F + g - kvj*robot->_dq;
            command_torques = (Jv.transpose())*F + g + N.transpose()*robot->_M*(-kvj*robot->_dq);

            //ofs << robot->_q(0) <<","<< robot->_q(1)   <<","<< robot->_q(2)   <<","<< robot->_q(3)   <<","<< robot->_q(4)  <<","<<  robot->_q(5)  <<","<<  robot->_q(6) << "\n";
            ofs << x.transpose() << "\n";
			
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
