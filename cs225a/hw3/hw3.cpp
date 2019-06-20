#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>

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
    ofs.open("h3.txt");
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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd g(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
 	MatrixXd J_0 = MatrixXd::Zero(3,dof);   
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);
    Vector3d vel = Vector3d::Zero();

	robot->Jv(Jv, link_name, pos_in_link);
	robot->J_0(J_0, link_name, pos_in_link);
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
		int controller_number = QUESTION_4;  

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
            Vector3d xd = Vector3d(0,0,0);
            Vector3d dxd = Vector3d(0,0,0);     
            Vector3d ddxd = Vector3d(0,0,0);
            Vector3d x = Vector3d::Zero();
            VectorXd qd(7);
            xd << (.3 + .1*sin(M_PI*time)),(.1+.1*cos(M_PI*time)), .5;
            dxd << (.1*M_PI*cos(M_PI*time)),(-.1*M_PI*sin(M_PI*time)), 0;
            ddxd << (-.1*M_PI*M_PI*sin(M_PI*time)),(-.1*M_PI*M_PI*cos(M_PI*time)), 0;
            qd << 0,0,0,0,0,0,0;

	        robot->Jv(Jv, link_name, pos_in_link);
	        robot->taskInertiaMatrix(Lambda, Jv);
	        robot->nullspaceMatrix(N, Jv);
            robot->position(x, link_name, pos_in_link);
            robot->linearVelocity(vel, link_name, pos_in_link);
            robot->gravityVector(g); 
            int kp = 100;
            int kv = 20; 
            int kpj = 50;
            int kvj = 14;
            VectorXd F(3);

            
            //F = Lambda*(-kp*(x - xd) - kv*(vel));
			//command_torques = Jv.transpose()*F + N.transpose()*(-kpj*(robot->_q-qd)-kvj*robot->_dq) + g;  
	        F = Lambda*(ddxd-kp*(x - xd) - kv*(vel - dxd));
			command_torques = Jv.transpose()*F + N.transpose()*(-kpj*(robot->_q-qd)-kvj*robot->_dq) + g;  
            ofs << xd.transpose() <<" " << x.transpose() <<"\n";
    }

    // ---------------------------  question 2 ---------------------------------------
    if(controller_number == QUESTION_2)
    {
        Vector3d xd = Vector3d(0,0,0);
        Vector3d x = Vector3d::Zero();
        VectorXd qd(7);
        //xd << -.1,.15,.2;
        xd << -.65,-.45,.7;

        robot->Jv(Jv, link_name, pos_in_link);
        robot->taskInertiaMatrix(Lambda, Jv);
        robot->nullspaceMatrix(N, Jv);
        robot->position(x, link_name, pos_in_link);
        robot->gravityVector(g); 
        robot->linearVelocity(vel, link_name, pos_in_link);
        int kp = 100;
        int kv = 20; 
        int kpj = 50;
        int k_damp = 14;
        VectorXd F(3);
        VectorXd gamma_m(7);
        gamma_m = robot->_q;
        gamma_m(3) = gamma_m(3) + 100;
        gamma_m(5) = gamma_m(5) - 105;

        F = Lambda*(-kp*(x - xd) - kv*(vel));
        //command_torques = Jv.transpose()*F+ N.transpose()*(-k_damp*robot->_dq)+g;
        //command_torques = Jv.transpose()*F+N.transpose()*gamma_m+ N.transpose()*(-k_damp*robot->_dq)+g;
        command_torques = Jv.transpose()*F+gamma_m+ N.transpose()*(-k_damp*robot->_dq)+g;
        ofs << time << "  " << xd.transpose() <<" " << x.transpose() << "  "<< time<< "  " <<robot->_q(3) << "  "<< robot->_q(5) << "\n";
               
    }

    // ---------------------------  question 3 ---------------------------------------
    if(controller_number == QUESTION_3)
    {
        Vector3d xd = Vector3d::Zero();
        Vector3d x = Vector3d::Zero();
        Vector3d w = Vector3d::Zero();
        Vector3d delta_phi = Vector3d::Zero();
        MatrixXd Lambda_0 = MatrixXd::Zero(6,6);
        MatrixXd N_0 = MatrixXd::Zero(dof,dof);
        Matrix3d Rd; 
        Matrix3d R;
        VectorXd qd(7);
        xd << .6,.3,.5;
        qd << 0,0,0,0,0,0,0;
        Rd << cos(M_PI/3),0,sin(M_PI/3),
                     0, 1, 0,
            -sin(M_PI/3),0,cos(M_PI/3);

        robot->Jv(Jv, link_name, pos_in_link);
        robot->J_0(J_0, link_name, pos_in_link);
        robot->taskInertiaMatrix(Lambda_0, J_0);
        robot->nullspaceMatrix(N_0, J_0);
        robot->position(x, link_name, pos_in_link);
        robot->linearVelocity(vel, link_name, pos_in_link);
        robot->angularVelocity(w,link_name);
            robot->gravityVector(g); 
            robot->rotation(R,link_name);
            Sai2Model::orientationError(delta_phi, Rd, R); 

            int kp = 50;
            int kv = 0; 
            int kvj = 0;

            VectorXd F1(6);
            MatrixXd  F(6,1);
            F1 << kp*(xd-x) - kv*vel, kp*(-delta_phi)-kv*w;
           	
            F = Lambda_0*(F1);
// this controller freaks out due to matrix issues
            command_torques = J_0.transpose()*F- N_0.transpose()*(-kvj*robot->_dq)+g;
          
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
            Vector3d xd = Vector3d(0,0,0);
            Vector3d x = Vector3d::Zero();
            VectorXd qd(7);
            Vector3d dxd = Vector3d::Zero();
           
            xd << .6,.3,.4;
            qd << 0,0,0,0,0,0,0;

	        robot->Jv(Jv, link_name, pos_in_link);
	        robot->taskInertiaMatrix(Lambda, Jv);
	        robot->linearVelocity(vel, link_name, pos_in_link);
            robot->nullspaceMatrix(N, Jv);
            robot->position(x, link_name, pos_in_link);
            robot->gravityVector(g); 
            float kp = 200;
            float kv = 30; 
            int kpj = 50;
            int kvj = 30;
            float mu = 0; 
            VectorXd F(3);

            //F = Lambda*(-kp*(x - xd) - kv*(vel));
            //command_torques = Jv.transpose()*F+ N.transpose()*robot->_M*(-kpj*(robot->_q-qd)-kvj*robot->_dq)+g;
 
            //sat
            dxd = (kp/kv)*(xd-x);
            float V = 0.1/(.0001+dxd.norm());
            if(abs(V) > 1) mu = V/(.0001+abs(V)); 
            else mu = V; 
            // mu = sat(.1/dxd.norm());
            F = Lambda*(-kv*(vel - mu*dxd));
            command_torques = Jv.transpose()*F+ N.transpose()*robot->_M*(-kpj*(robot->_q-qd)-kvj*robot->_dq)+g;
            ofs << time << "  " << xd.transpose() <<" " << x.transpose() << "  "<< time << "  " <<vel.transpose() << "\n";
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
