#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <bits/stdc++.h> 
#include <chrono>
#include <fstream>

using namespace Eigen;
using namespace std;

// Declare structs
struct Cost_values
{
    MatrixXd h;
    VectorXd f;
};

void forward_kinematics(Vector3d& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg);
void analytic_jacobian(Matrix3d& jacobian, const Vector3d& q, const Vector3d& current_pos, const Vector3d& posb, const Vector3d& rotb, const int& leg);

class Legs
{
    public:
    Vector3d leg1, leg2, leg3, leg4, posb, rotb, com;
    Matrix3d leg1_jacobian, leg2_jacobian, leg3_jacobian, leg4_jacobian;

    void update_position(const VectorXd& q)
    {
        posb = q.segment(0, 3);
        rotb = q.segment(15, 3);
        forward_kinematics(leg1, q.segment(3, 3), posb, rotb, 1);
        forward_kinematics(leg2, q.segment(6, 3), posb, rotb, 2);
        forward_kinematics(leg3, q.segment(9, 3), posb, rotb, 3);
        forward_kinematics(leg4, q.segment(12, 3), posb, rotb, 4);
    }

    void update_jacobian(const VectorXd& q) 
    {
        analytic_jacobian(leg1_jacobian, q.segment(3, 3), leg1, posb, rotb, 1);
        analytic_jacobian(leg2_jacobian, q.segment(6, 3), leg2, posb, rotb, 2);
        analytic_jacobian(leg3_jacobian, q.segment(9, 3), leg3, posb, rotb, 3);
        analytic_jacobian(leg4_jacobian, q.segment(12, 3), leg4, posb, rotb, 4);
    }
};

class Parameters
{
public:
    // Maximum number of iterations
    int max_iter;
    // Gain of quadratic function
    int lamb;
    // Time between signals
    double d_t;
    // Maximum tolerance for minimization
    double tol;
    // Weights: [posb, leg1, leg2, leg3, leg4, rotb]
    VectorXi w;

    Parameters() {
        // Maximum number of iterations
        this->max_iter = 100;
        // Gain of quadratic function
        this->lamb = -10;
        // Time between signals
        this->d_t = 0.01;
        // Maximum tolerance for minimization
        this->tol = 0.01;
        // Weights: [posb, leg1, leg2, leg3, leg4, rotb]
        this->w = VectorXi::Ones(7);
        this->w(0) = 1;
        this->w(5) = 1;
        this->w(6) = 0;
    }
};

// Declare functions
void initialize(VectorXd& q);
void center(VectorXd& q);
void denavit_hartenberg(Matrix4d& mat, const double& theta, const double& alpha, const double& r, const double& d);
void reposition_leg(Matrix4d& leg_pos, const double& ang, const double& dx, const double& dy);
void q2rot(Matrix3d& rotm, const double& a, const double& b, const double& c);
void rot2q(Vector3d& values, const Matrix3d& rotm);
void quad_prog(MatrixXd& qf, VectorXd& q, const VectorXd& xd, const Parameters& parameters, const Vector3d& com_xd, Legs& pos);
void costfunc(Cost_values& cost, const VectorXd& q, const VectorXd& xd, const int& lamb, const VectorXi& w, const VectorXd& com_xd, Legs& pos);
void leg_jacobian(MatrixXd& leg_jacobian, const Vector3d& q, const int& leg, Legs& pos);
void calc_err(double& err, const VectorXd& q, const VectorXd& xd, Legs& pos);
void com_pos(Vector3d& com_pos, const VectorXd& q);
void com_kinematics(VectorXd& position, const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb, const Vector4d& w);
void com_jacobian(MatrixXd& com_jacobian, const VectorXd& q, const Vector3d& current_pos);
void full_kinematics(VectorXd& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg);
void compare(const VectorXd& xd, const Vector3d& com_xd, const Legs& pos);

// Functions for walking
void gait_step(double& modulo, double& residuo, double& num, double& distance);
void walking(VectorXd& posd, Legs& pos, MatrixXd& final, int& ind);

// Functions for joystick control
void print_position(MatrixXd& qf, ofstream& myfile, const Parameters& parameters);
void print_configuration(MatrixXd& qf, ofstream& myfile, const Parameters& parameters);

// Constants
const double pi = M_PI;

int main(int argc, char **argv) {
    // Se inician los nodos y topicos de ROS
    ros::init(argc, argv, "sendJointsGzNode");
    ros::start();
    ros::NodeHandle n;

    // Se publica el espacio articular actual del robot
    ros::Publisher pub_leg1m1 = n.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
    ros::Publisher pub_leg1m2 = n.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 10);
    ros::Publisher pub_leg1m3 = n.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command", 10);
    ros::Publisher pub_leg2m1 = n.advertise<std_msgs::Float64>("/robot/joint4_position_controller/command", 10);
    ros::Publisher pub_leg2m2 = n.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 10);
    ros::Publisher pub_leg2m3 = n.advertise<std_msgs::Float64>("/robot/joint6_position_controller/command", 10);
    ros::Publisher pub_leg3m1 = n.advertise<std_msgs::Float64>("/robot/joint7_position_controller/command", 10);
    ros::Publisher pub_leg3m2 = n.advertise<std_msgs::Float64>("/robot/joint8_position_controller/command", 10);
    ros::Publisher pub_leg3m3 = n.advertise<std_msgs::Float64>("/robot/joint9_position_controller/command", 10);
    ros::Publisher pub_leg4m1 = n.advertise<std_msgs::Float64>("/robot/joint10_position_controller/command", 10);
    ros::Publisher pub_leg4m2 = n.advertise<std_msgs::Float64>("/robot/joint11_position_controller/command", 10);
    ros::Publisher pub_leg4m3 = n.advertise<std_msgs::Float64>("/robot/joint12_position_controller/command", 10);
    ros::Publisher pub_posbx = n.advertise<std_msgs::Float64>("/robot/joint13_position_controller/command", 10);
    ros::Publisher pub_posby = n.advertise<std_msgs::Float64>("/robot/joint14_position_controller/command", 10);
    ros::Publisher pub_posbz = n.advertise<std_msgs::Float64>("/robot/joint15_position_controller/command", 10);
    ros::Publisher pub_rotbx = n.advertise<std_msgs::Float64>("/robot/joint16_position_controller/command", 10);
    ros::Publisher pub_rotby = n.advertise<std_msgs::Float64>("/robot/joint17_position_controller/command", 10);
    ros::Publisher pub_rotbz = n.advertise<std_msgs::Float64>("/robot/joint18_position_controller/command", 10);

    // Se publica el espacio de configuracion deseado del robot
    ros::Publisher pub_desired = n.advertise<std_msgs::Float64MultiArray>("/desired_pos", 10);
    ros::Publisher pub_rviz = n.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate loop_rate(3.5);

    // Se declaran los mensajes de ROS
    std_msgs::Float64 j1;
    std_msgs::Float64 j2;
    std_msgs::Float64 j3;
    std_msgs::Float64 j4;
    std_msgs::Float64 j5;
    std_msgs::Float64 j6;
    std_msgs::Float64 j7;
    std_msgs::Float64 j8;
    std_msgs::Float64 j9;
    std_msgs::Float64 j10;
    std_msgs::Float64 j11;
    std_msgs::Float64 j12;
    std_msgs::Float64 j13;
    std_msgs::Float64 j14;
    std_msgs::Float64 j15;
    std_msgs::Float64 j16;
    std_msgs::Float64 j17;
    std_msgs::Float64 j18;
    std_msgs::Float64MultiArray jdesired;
    sensor_msgs::JointState rviz_val;

    // Se asignan las variables 
    VectorXd q(18), xd(18);
    Vector3d com_xd;
    Parameters parameters;
    MatrixXd qf(parameters.max_iter, 18);
    Legs pos;

    initialize(q);
    pos.update_position(q);
    pos.update_jacobian(q);

    // Desired position[posb, leg1, leg2, leg3, leg4, rotb]:
    //xd.segment(0, 3)  << 3, 2, 0;
    //xd.segment(3, 3)  << 3 + pos.leg1(0), 2 + pos.leg1(1), 0 + pos.leg1(2);
    //xd.segment(6, 3)  << 1 + pos.leg2(0), 0 + pos.leg2(1), 0 + pos.leg2(2);
    //xd.segment(9, 3)  << 0 + pos.leg3(0), 1 + pos.leg3(1), 0 + pos.leg3(2);
    //xd.segment(12, 3) << 0 + pos.leg4(0), 0 + pos.leg4(1), 1 + pos.leg4(2);
    //xd.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;

    xd.segment(0, 3)  << 0, 0, 0;
    xd.segment(3, 3)  << pos.leg1(0), pos.leg1(1), pos.leg1(2) + 2;
    xd.segment(6, 3)  << pos.leg2(0), pos.leg2(1), pos.leg2(2) + 2;
    xd.segment(9, 3)  << pos.leg3(0), pos.leg3(1), pos.leg3(2) + 2;
    xd.segment(12, 3) << pos.leg4(0), pos.leg4(1), pos.leg4(2) + 2;
    xd.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;

    // Desired position of COM
    //com_xd = cmass(q);
    //com_xd << 2, 0.8, -3.8;
    com_xd << 0, 0, 0;
    
    // Se prepara la data de posiciones actuales
    std::vector<string> names; 
    names.push_back("leg1_motor1");
    names.push_back("leg2_motor1");
    names.push_back("leg3_motor1");
    names.push_back("leg4_motor1");  
    names.push_back("leg1_motor2");
    names.push_back("leg2_motor2");
    names.push_back("leg3_motor2");
    names.push_back("leg4_motor2");
    names.push_back("leg1_motor3");
    names.push_back("leg2_motor3");
    names.push_back("leg3_motor3");
    names.push_back("leg4_motor3");
    // Se inserta la data en el array
    rviz_val.name.clear();
    for (int i = 0; i < names.size(); i++)
    {
        rviz_val.name.push_back(names[i]);
    }
    
    int count = 0;

    // Valores para la caminata
    int ind = 0;
    VectorXd posd(4);
    cout << "Posicion actual: " << endl;
    cout << " xa: ";
    cin >> posd(0);
    cout << " ya: ";
    cin >> posd(1);
    cout << "Posicion deseada: " << endl;
    cout << " xd: ";
    cin >> posd(2);
    cout << " yd: ";
    cin >> posd(3);
    MatrixXd final(18,300);
    walking(posd, pos, final, ind);

    // Se escriben los valores en un archivo
    ofstream myfile;
    myfile.open ("charlyposgait.txt");

    while (ros::ok()) {  
        //cout << "Numero de paso" << endl;
        //cin >> count;
        //auto start = std::chrono::system_clock::now();
        // Quadratic program
        //quad_prog(qf, q, xd, parameters, com_xd, pos);
        quad_prog(qf, q, final.col(count), parameters, com_xd, pos);
        
        //auto end = std::chrono::system_clock::now();
        //std::chrono::duration<float,std::milli> duration = end - start;
        //std::cout << duration.count() << "s " << endl;
        
        // Compare desired and current positions(x, y, z)
        //compare(xd, com_xd, pos);
        //cout << final.col(count) << endl;
        //compare(final.col(count), com_xd, pos);
        //cout << endl << q << endl;

        j1.data = q(3);
        j2.data = q(4);
        j3.data = q(5);
        j4.data = q(6);
        j5.data = q(7);
        j6.data = q(8);
        j7.data = q(9);
        j8.data = q(10);
        j9.data = q(11);
        j10.data = q(12);
        j11.data = q(13);
        j12.data = q(14);
        j13.data = q(0);
        j14.data = q(1);
        j15.data = q(2);
        j16.data = q(15);
        j17.data = q(16);
        j18.data = q(17);
        
        // Se prepara la data de posicion deseada
        vector<double> rosxd;
		rosxd.resize(final.col(count).size());
		VectorXd::Map(&rosxd[0], final.col(count).size()) = final.col(count);

		// Se inserta la data en el array
		jdesired.data.clear();
		for (int i = 0; i < rosxd.size(); i++)
		{
			jdesired.data.push_back(rosxd[i]);
		}

        // Se prepara la data de configuracion actual
        vector<double> rvizq;
        rvizq.resize(q.size());
        VectorXd::Map(&rvizq[0], q.size()) = q;

        // Se inserta la data en el array
        rviz_val.position.clear();
        rviz_val.header.stamp = ros::Time::now();
        rviz_val.position.push_back(rvizq[3]);
        rviz_val.position.push_back(rvizq[6]);
        rviz_val.position.push_back(rvizq[9]);
        rviz_val.position.push_back(rvizq[12]);
        rviz_val.position.push_back(rvizq[4]);
        rviz_val.position.push_back(rvizq[7]);
        rviz_val.position.push_back(rvizq[10]);
        rviz_val.position.push_back(rvizq[13]);
        rviz_val.position.push_back(rvizq[5]);
        rviz_val.position.push_back(rvizq[8]);
        rviz_val.position.push_back(rvizq[11]);
        rviz_val.position.push_back(rvizq[14]);

		// Se publican los valores del espacio articular
        pub_leg1m1.publish(j1);
        pub_leg1m2.publish(j2);
        pub_leg1m3.publish(j3);
        pub_leg2m1.publish(j4);
        pub_leg2m2.publish(j5);
        pub_leg2m3.publish(j6);
        pub_leg3m1.publish(j7);
        pub_leg3m2.publish(j8);
        pub_leg3m3.publish(j9);
        pub_leg4m1.publish(j10);
        pub_leg4m2.publish(j11);
        pub_leg4m3.publish(j12);
        pub_posbx.publish(j13);
        pub_posby.publish(j14);
        pub_posbz.publish(j15);
        pub_rotbx.publish(j16);
        pub_rotby.publish(j17);
        pub_rotbz.publish(j18);
        pub_desired.publish(jdesired);
        pub_rviz.publish(rviz_val);

        ros::spinOnce();
        loop_rate.sleep();
        
        if (count == ind - 1) {
            count = 0;
            posd(0) = posd(2);
            posd(1) = posd(3);
            cout << endl << "Posicion actual: " << endl;
            cout << " xa: " << posd(0) << endl;
            cout << " ya: " << posd(1) << endl;
            cout << "Posicion deseada: " << endl;
            cout << " xd: ";
            cin >> posd(2);
            cout << " yd: ";
            cin >> posd(3);
            walking(posd, pos, final, ind);
        }
        if (count < ind - 1) {
            // print_position(qf, myfile, parameters);
            count++;
        }
    }
    myfile.close();
}


void print_position(MatrixXd& qf, ofstream& myfile, const Parameters& parameters) {
    Legs posprint;
    int i = 0;
    while (qf(i,4) != 0) {
        posprint.update_position(qf.row(i));
        for (int j = 0; j < 3; j++) {
            myfile << posprint.posb(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg1(j) << ',';  
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg2(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg3(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.leg4(j) << ',';
        }
        for (int j = 0; j < 3; j++) {
            myfile << posprint.rotb(j);
            if (j < 2) {
                myfile << ',';  
            } 
        }
        myfile << '\n';
        i++;
    }
}

void print_configuration(MatrixXd& qf, ofstream& myfile, const Parameters& parameters) {
    int i = 0;
    while (qf(i,4) != 0) {
        for (int j = 0; j < 18; j++) {
            myfile << qf(i,j);
            if (j < 17) {
                myfile << ',';  
            } 
        }
        myfile << '\n';
        i++;
    }
}


void gait_step(double& modulo, double& residuo, double& num, double& distance) {
    if (distance >= 8) {
        modulo = 8;
        double r = fmod(distance, modulo);

        if (r >= 1) {
            residuo = r;
        }
        else {
            residuo = 0;
        }
        
        num = floor((distance - residuo)/modulo);
    }
    if (distance >= 1 && distance < 8) {
        modulo = 1;
        double r = fmod(distance, modulo);
        
        if (r >= 1) {
            residuo = r;
        }
        else {
            residuo = 0;
        }
    
        num = floor((distance-residuo)/modulo);
    }
    if (distance < 1) {
        modulo = 0;
        residuo = 0;
        num = 0;
    }
}


void walking(VectorXd& posd, Legs& pos, MatrixXd& final, int& ind) {
    // Initialize variables
    int quadrant;
    double angle, distance;
    double modulo, residuo, num, resix, resiy;
    VectorXd M(18);
    
    // Find the direction vector
    angle = atan2(posd(3) - posd(1), posd(2) - posd(0));
    distance = sqrt(pow(posd(2) - posd(0),2) + pow(posd(3) - posd(1),2));

    // Number of steps required
    gait_step(modulo, residuo, num, distance);
    
    // Desplazamiento en funcion del angulo hallado
    double parax = modulo * cos(angle);
    double paray = modulo * sin(angle);

    // Center legs
    Vector3d cl1, cl2, cl3, cl4;
    VectorXd qc(12);
    center(qc);

    forward_kinematics(cl1, qc.segment(0, 3), pos.posb, pos.rotb, 1);
    forward_kinematics(cl2, qc.segment(3, 3), pos.posb, pos.rotb, 2);
    forward_kinematics(cl3, qc.segment(6, 3), pos.posb, pos.rotb, 3);
    forward_kinematics(cl4, qc.segment(9, 3), pos.posb, pos.rotb, 4);

    M.segment(0, 3)  << pos.posb(0), pos.posb(1), pos.posb(2);
    M.segment(3, 3)  << cl1(0), cl1(1), cl1(2);
    M.segment(6, 3)  << cl2(0), cl2(1), cl2(2);
    M.segment(9, 3)  << cl3(0), cl3(1), cl3(2);
    M.segment(12, 3) << cl4(0), cl4(1), cl4(2);
    M.segment(15, 3) << pos.rotb(0), pos.rotb(1), pos.rotb(2);
    final.col(0) = M;
    
    // Select working quadrant
    if (angle >= 0 && angle < M_PI / 2) {
        quadrant = 1;
    }
    else if (angle >= M_PI / 2 && angle <= M_PI) {
        quadrant = 2;
    }
    else if (angle >= - M_PI && angle < - M_PI / 2) {
        quadrant = 3;
    }
    else if (angle >= - M_PI / 2 && angle < 0) {
        quadrant = 4; 
    }

    // Se selecciona el cuadrante sobre el cual se trabaja
    switch(quadrant) {
        case 1:
            // Levanta pata 2 y la mueve a la posicion deseada
            M(8) = -20;
            final.col(1) = M;
            M(6) = M(6) + parax/2;
            M(7) = M(7) + paray/2;
            final.col(2) = M;
            M(8) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 3
            M(11) = -20;
            final.col(5) = M;
            M(9) = M(9) + parax/2;
            M(10) = M(10) + paray/2;
            final.col(6) = M;
            M(11) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+1) = M;
                M(5) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind+4) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+5) = M;
                M(14) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+7) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+8) = M;
                M(8) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind+11) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+12) = M;
                M(11) = -22.5;
                final.col(ind+13) = M;
                  
                ind += 14;
            }
            break;

        case 2:
            // Levanta pata 4 y la mueve a la posicion deseada
            M(14) = -20;
            final.col(1) = M;
            M(12) = M(12) + parax/2;
            M(13) = M(13) + paray/2;
            final.col(2) = M;
            M(14) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 1
            M(5) = -20;
            final.col(5) = M;
            M(3) = M(3) + parax/2;
            M(4) = M(4) + paray/2;
            final.col(6) = M;
            M(5) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+1) = M;
                M(11) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+4) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+5) = M;
                M(8) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind+7) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+8) = M;
                M(14) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind+11) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+12) = M;
                M(5) = -22.5;
                final.col(ind+13) = M;
                  
                ind += 14;
            }
            break;

        case 3:
            // Levanta pata 3 y la mueve a la posicion deseada
            M(11) = -20;
            final.col(1) = M;
            M(9) = M(9) + parax/2;
            M(10) = M(10) + paray/2;
            final.col(2) = M;
            M(11) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 2
            M(8) = -20;
            final.col(5) = M;
            M(6) = M(6) + parax/2;
            M(7) = M(7) + paray/2;
            final.col(6) = M;
            M(8) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+1) = M;
                M(14) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind+4) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+5) = M;
                M(5) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind+7) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+8) = M;
                M(11) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+11) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+12) = M;
                M(8) = -22.5;
                final.col(ind+13) = M;
                  
                ind += 14;
            }
            break;
             
        case 4:
            // Levanta pata 1 y la mueve a la posicion deseada
            M(5) = -20;
            final.col(1) = M;
            M(3) = M(3) + parax/2;
            M(4) = M(4) + paray/2;
            final.col(2) = M;
            M(5) = -22.5;
            final.col(3) = M;

            // MUEVE EL CENTRO DE MASAS
            M(0) = M(0) + parax/4;
            M(1) = M(1) + paray/4;
            final.col(4) = M;

            // LEVANTA Y MUEVE LA PATA 4
            M(14) = -20;
            final.col(5) = M;
            M(12) = M(12) + parax/2;
            M(13) = M(13) + paray/2;
            final.col(6) = M;
            M(14) = -22.5;
            final.col(7) = M;

            ind = 8; // Indice de la matriz IMPORTANTE
            
            for (int n=0; n < num; n++) {
                // LEVANTA Y MUEVE LA PATA 3
                M(11) = -20;
                final.col(ind) = M;
                M(9) = M(9) + parax;
                M(10) = M(10) + paray;
                final.col(ind+1) = M;
                M(11) = -22.5;
                final.col(ind+2) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+3) = M;
                
                // LEVANTA Y MUEVE LA PATA 2
                M(8) = -20;
                final.col(ind+4) = M;
                M(6) = M(6) + parax;
                M(7) = M(7) + paray;
                final.col(ind+5) = M;
                M(8) = -22.5;
                final.col(ind+6) = M;
                
                // LEVANTA Y MUEVE LA PATA 1
                M(5) = -20;
                final.col(ind+7) = M;
                M(3) = M(3) + parax;
                M(4) = M(4) + paray;
                final.col(ind+8) = M;
                M(5) = -22.5;
                final.col(ind+9) = M;
                
                // MUEVE EL CENTRO DE MASAS
                M(0) = M(0) + parax/2;
                M(1) = M(1) + paray/2;
                final.col(ind+10) = M;

                // LEVANTA Y MUEVE LA PATA 4
                M(14) = -20;
                final.col(ind+11) = M;
                M(12) = M(12) + parax;
                M(13) = M(13) + paray;
                final.col(ind+12) = M;
                M(14) = -22.5;
                final.col(ind+13) = M;
                
                ind += 14;
            }
            break;
    }

}


void initialize(VectorXd& q) {
    // q : Initial configuration vector[posb, leg1, leg2, leg3, leg4, rotb]
    q.segment(0, 3) << 0, 0, 0;
    q.segment(3, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(6, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(9, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(12, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;
}


void center(VectorXd& q) {
    q.segment(0, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(3, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(6, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
    q.segment(9, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
}


void forward_kinematics(Vector3d& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    // This function finds the forward kinematics of each leg of the robot.
    // Returns the position of the end - effector according to the position and orientation of the base.This is possible
    // by calculating the jacobian of each leg a locating it in a homogeneous transformation matrix.
    
    Matrix3d rotm;
    Matrix4d leg_end, m_link1, m_link2, m_link3, trans, repos_leg;
    const double r1 = 5.5;     // Distance from servo 1 to 2
    const double r2 = 7.5;     // Distance from servo 2 to 3
    const double r3 = 22.5;    // Distance from servo 3 to effector
    const double r4 = 10.253;  // Distance from base to servo 1

    // Denavit - Hartenberg matrices
    denavit_hartenberg(m_link1, q(0), pi / 2, r1, 0);
    denavit_hartenberg(m_link2, q(1) + pi / 2, pi, -r2, 0);
    denavit_hartenberg(m_link3, q(2), pi, -r3, 0);

    // Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    q2rot(rotm, rotb(0), rotb(1), rotb(2));

    trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
             rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
             rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
             0, 0, 0, 1;

    // Position of the legs with respect to the base
    if (leg == 1) {
        reposition_leg(repos_leg, -3 * pi / 4, r4, -r4);
    }
    else if (leg == 2) {
        reposition_leg(repos_leg, -pi / 4, r4, r4);
    }
    else if (leg == 3) {
        reposition_leg(repos_leg, 3 * pi / 4, -r4, -r4);
    }
    else {
        reposition_leg(repos_leg, pi / 4, -r4, r4);
    }
    
    // End - effector position(x, y, z)
    leg_end << trans * repos_leg*m_link1*m_link2*m_link3;
    pos << leg_end(0, 3), leg_end(1, 3), leg_end(2, 3);
}


void denavit_hartenberg(Matrix4d& mat, const double& theta, const double& alpha, const double& r, const double& d) {
    mat << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), r * cos(theta),
           sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta),
           0.0, sin(alpha), cos(alpha), d,
           0.0, 0.0, 0.0, 1.0;
}


void reposition_leg(Matrix4d& leg_pos, const double& ang, const double& dx, const double& dy) {
    leg_pos << cos(ang), -sin(ang), 0, dx,
               sin(ang),  cos(ang), 0, dy,
               0, 0, 1, 0,
               0, 0, 0, 1;
}


void q2rot(Matrix3d& rotm, const double& a, const double& b, const double& c) {
    Matrix3d rx, ry, rz;
    
    rx << 1, 0, 0,
          0, cos(a), -sin(a),
          0, sin(a), cos(a);

    ry << cos(b), 0, sin(b),
          0, 1, 0,
          -sin(b), 0, cos(b);

    rz << cos(c), -sin(c), 0,
         sin(c), cos(c), 0,
         0, 0, 1;

    rotm << rz*ry*rx;
}


void rot2q(Vector3d& values, const Matrix3d& rotm) {
    double a = atan2(rotm(2,1), rotm(2,2));
    double b = atan2(-rotm(2,0), sqrt(pow(rotm(2,1),2)) + (pow(rotm(2,2),2)));
    double c = atan2(rotm(1,0), rotm(0,0));
    values << a, b, c;
}


void analytic_jacobian(Matrix3d& jacobian, const Vector3d& q, const Vector3d& current_pos, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    // Analytic jacobian for leg position. Returns a 3x3 matrix with input q = [q1, q2, q3]
    Vector3d dq;
    Vector3d pos_dq;
    double delta = 0.00001;

    for (int i = 0; i <= 2; i++) {
        // Copy initial articular configuration and use delta increment on index i
        dq = q;
        dq(i) += delta;
        
        // Homegeneous Transformation Matrix after increment
        if (leg == 1) {
            forward_kinematics(pos_dq, dq, posb, rotb, 1);
        }
        else if (leg == 2) {
            forward_kinematics(pos_dq, dq, posb, rotb, 2);
        }
        else if (leg == 3) {
            forward_kinematics(pos_dq, dq, posb, rotb, 3);
        }
        else {
            forward_kinematics(pos_dq, dq, posb, rotb, 4);
        }

        // Finite difference
        jacobian(0, i) = (pos_dq(0) - current_pos(0)) / delta;
        jacobian(1, i) = (pos_dq(1) - current_pos(1)) / delta;
        jacobian(2, i) = (pos_dq(2) - current_pos(2)) / delta;
    }
}


void quad_prog(MatrixXd& qf, VectorXd& q, const VectorXd& xd, const Parameters& parameters, const Vector3d& com_xd, Legs& pos) {
    // This function manages the minimization program and find the error of the desired function
    
    USING_NAMESPACE_QPOASES
    
    int itr = 0;
    int j = 0;
    double err;
    double arr_h[324];
    double arr_f[18];
    Vector3d rot_vec;
    VectorXd dq(18);
    Matrix3d rot_axis, skw, rgs, rot;
    Cost_values cost;

    // Setting up QProblemB object
    QProblemB qp_quad(18);

    Options options;
    options.initialStatusBounds = ST_INACTIVE;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;
    options.printLevel = PL_NONE;
    qp_quad.setOptions(options);

    while (itr < parameters.max_iter) {
        costfunc(cost, q, xd, parameters.lamb, parameters.w, com_xd, pos);
        Map<MatrixXd>(&arr_h[0], 18, 18) = cost.h;
        Map<VectorXd>(&arr_f[0], 18, 1) = cost.f;

        // Setup data of first QP
        real_t H[18 * 18];
        for (int i = 0; i < 324; ++i) {
            H[i] = arr_h[i];
        }
        real_t g[18];
        for (int i = 0; i < 18; ++i) {
            g[i] = arr_f[i];
        }
        real_t lb[18];
        for (int i = 0; i < 18; ++i) {
            lb[i] = -1e10;
        }
        real_t ub[18];
        for (int i = 0; i < 18; ++i) {
            ub[i] = 1e10;
        }

        // Solve QP
        int_t nWSR = 10;
        qp_quad.init(H, g, lb, ub, nWSR, 0);
        real_t xOpt[18];

        qp_quad.getPrimalSolution(xOpt);
        for (int i = 0; i < 18; ++i) {
            dq[i] = xOpt[i];
        }

        // Update the position vector
        q.segment(0, 15) = q.segment(0, 15) + parameters.d_t * dq.segment(0, 15);

        // Update the orientation vector
        rot_axis << cos(q(16)) * cos(q(17)), -sin(q(17)), 0,
                    cos(q(16)) * sin(q(17)),  cos(q(17)), 0,
                                -sin(q(16)),           0, 1;

        rot_vec << rot_axis * dq.segment(15, 3);
        
        skw <<           0, -rot_vec(2),  rot_vec(1),
                rot_vec(2),           0, -rot_vec(0),
               -rot_vec(1),  rot_vec(0),           0;

        // Rodrigues rotation formula
        rgs = Matrix3d::Identity() + sin(parameters.d_t) * skw + (1 - cos(parameters.d_t)) * (skw*skw);
        q2rot(rot, q(15), q(16), q(17));
        rot *= rgs;
        rot2q(rot_vec, rot);
        q.segment(15, 3) = rot_vec;

        for (int i = 0; i < 18; ++i) {
            qf(j, i) = q(i);
        }

        calc_err(err, q, xd, pos);
        if (err <= parameters.tol) {
            itr++;
            break;
        }

        pos.update_position(q);
        pos.update_jacobian(q);
        itr++;
        j++;
    }
}


void costfunc(Cost_values& cost, const VectorXd& q, const VectorXd& xd, const int& lamb, const VectorXi& w, const VectorXd& com_xd, Legs& pos) {
    // This function finds the values of h and f in order to initialize the quadratic program.
    // The inputs are q : actuated and sub - actuated angles, xd : desired position vector, p : weights and lamb : gain.

    // Position and orientation of the base
    MatrixXd j1(3, 18), j2(3, 18), j3(3, 18), j4(3, 18), j5(3, 18), j6(3, 18), j_com(3, 18);
    
    // Jacobian of each leg 
    leg_jacobian(j1, q.segment(3, 3), 1, pos);
    leg_jacobian(j2, q.segment(6, 3), 2, pos);
    leg_jacobian(j3, q.segment(9, 3), 3, pos);
    leg_jacobian(j4, q.segment(12, 3), 4, pos);

    // Jacobians of the base position and orientation
    j5 << Matrix3d::Identity(), Matrix<double, 3, 15>::Zero();
    j6 << Matrix<double, 3, 15>::Zero(), Matrix3d::Identity();

    // Position and jacobian of center of mass
    com_pos(pos.com, q);
    //com_jacobian(j_com, q, pos.com);

    // Values of h and f(Hessian and vector of linear elements) :
    //MatrixXd h = w(0) * j1.transpose()*j1 + w(1) * j2.transpose()*j2 + w(2) * j3.transpose()*j3 + w(3) * j4.transpose()*j4 + w(4) * j5.transpose()*j5 + w(5) * j6.transpose()*j6 + w(6) * j_com.transpose()*j_com;
    MatrixXd h = w(0) * j1.transpose()*j1 + w(1) * j2.transpose()*j2 + w(2) * j3.transpose()*j3 + w(3) * j4.transpose()*j4 + w(4) * j5.transpose()*j5 + w(5) * j6.transpose()*j6;

    //VectorXd f = -2 * (w(1) * lamb * (pos.leg1 - xd.segment(3, 3)).transpose()*j1 + w(2) * lamb * (pos.leg2 - xd.segment(6, 3)).transpose()*j2 + w(3) * lamb * (pos.leg3 - xd.segment(9, 3)).transpose()*j3 + w(4) * lamb * (pos.leg4 - xd.segment(12, 3)).transpose()*j4 + w(0) * lamb * (pos.posb - xd.segment(0, 3)).transpose()*j5 + w(5) * lamb * (pos.rotb - xd.segment(15, 3)).transpose()*j6 + w(6) * lamb * (pos.com - com_xd).transpose()*j_com);
    VectorXd f = -2 * (w(1) * lamb * (pos.leg1 - xd.segment(3, 3)).transpose()*j1 + w(2) * lamb * (pos.leg2 - xd.segment(6, 3)).transpose()*j2 + w(3) * lamb * (pos.leg3 - xd.segment(9, 3)).transpose()*j3 + w(4) * lamb * (pos.leg4 - xd.segment(12, 3)).transpose()*j4 + w(0) * lamb * (pos.posb - xd.segment(0, 3)).transpose()*j5 + w(5) * lamb * (pos.rotb - xd.segment(15, 3)).transpose()*j6);

    cost = { h, f };
}


void leg_jacobian(MatrixXd& leg_jacobian, const Vector3d& q, const int& leg, Legs& pos) {
    Matrix3d jacobian, m_skew;
    
    if (leg == 1) {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg1;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
                  d(2), 0, -d(0),
                  -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), pos.leg1_jacobian, Matrix<double, 3, 9>::Identity(), m_skew;
    }
    else if (leg == 2) {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg2;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
            d(2), 0, -d(0),
            -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), Matrix3d::Zero(), pos.leg2_jacobian, Matrix<double, 3, 6>::Identity(), m_skew;
    }
    else if (leg == 3) {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg3;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
            d(2), 0, -d(0),
            -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 6>::Identity(), pos.leg3_jacobian, Matrix3d::Zero(), m_skew;
    }
    else {
        // Distance vector of the end - effector with respect to base
        Vector3d d = pos.posb - pos.leg4;

        // Skew - Symmetric matrix
        m_skew << 0, -d(2), d(1),
            d(2), 0, -d(0),
            -d(1), d(0), 0;

        leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 9>::Identity(), pos.leg4_jacobian, m_skew;
    }
}


void calc_err(double& err, const VectorXd& q, const VectorXd& xd, Legs& pos) {
    // Find the error of each leg and the base
    Vector3d err_leg1 = xd.segment(3, 3) - pos.leg1;
    Vector3d err_leg2 = xd.segment(6, 3) - pos.leg2;
    Vector3d err_leg3 = xd.segment(9, 3) - pos.leg3;
    Vector3d err_leg4 = xd.segment(12, 3) - pos.leg4;
    Vector3d err_posb = xd.segment(0, 3) - pos.posb;
    Vector3d err_rotb = xd.segment(15, 3) - pos.rotb;

    // Sum of the squared errors
    err = sqrt(pow(err_leg1(0), 2) + pow(err_leg1(1), 2) + pow(err_leg1(2), 2) + pow(err_leg2(0), 2) + pow(err_leg2(1), 2) + pow(err_leg2(2), 2) 
        + pow(err_leg3(0), 2) + pow(err_leg3(1), 2) + pow(err_leg3(2), 2) + pow(err_leg4(0), 2) + pow(err_leg4(1), 2) + pow(err_leg4(2), 2) 
        + pow(err_posb(0), 2) + pow(err_posb(1), 2) + pow(err_posb(2), 2) + pow(err_rotb(0), 2) + pow(err_rotb(1), 2) + pow(err_rotb(2), 2));
}


void com_pos(Vector3d& com_pos, const VectorXd& q) {
    // Weights
    double w_com1 = 0.075;
    double w_com2 = 0.15;
    double w_com3 = 0.2;
    double w_base = 0.7;
    double w_total = 4 * w_com1 + 4 * w_com2 + 4 * w_com3 + w_base;
    Vector4d w;
    VectorXd leg1_com(12), leg2_com(12), leg3_com(12), leg4_com(12);
    Vector3d posb, rotb;
    posb << q.segment(0, 3);
    rotb << q.segment(15, 3);

    w << w_com1, w_com2, w_com3, w_total;

    // Find the center of mass
    com_kinematics(leg1_com, q.segment(3, 3), 1, posb, rotb, w);
    com_kinematics(leg2_com, q.segment(6, 3), 2, posb, rotb, w);
    com_kinematics(leg3_com, q.segment(9, 3), 3, posb, rotb, w);
    com_kinematics(leg4_com, q.segment(12, 3), 4, posb, rotb, w);

    // COM of the base
    Vector3d base = (w_base / w_total) * posb;

    // COM position
    com_pos << leg1_com.segment(0, 3) + leg1_com.segment(3, 3) + leg1_com.segment(6, 3) + leg2_com.segment(0, 3) + leg2_com.segment(3, 3) 
        + leg2_com.segment(6, 3) + leg3_com.segment(0, 3) + leg3_com.segment(3, 3) + leg3_com.segment(6, 3) + leg4_com.segment(0, 3) 
        + leg4_com.segment(3, 3) + leg4_com.segment(6, 3) + base;
}


void com_kinematics(VectorXd& position, const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb, const Vector4d& w) {
    // Variables
    const double r1 = 5.5;     // Distance from servo 1 to 2
    const double r2t = 3.75;   // Distance from servo 2 to com2
    const double r2 = 7.5;     // Distance from servo 2 to 3
    const double r3t = 11;     // Distance from servo 3 to com3
    const double r3 = 22.5;    // Distance from servo 3 to end - effector
    const double r4 = 10.253;  // Distance from base to servo 1
    Matrix3d rotm;
    Matrix4d m_1, m_2t, m_2, m_3t, m_3, trans, repos_leg;

    // Denavit - Hartenberg matrices
    denavit_hartenberg(m_1, q(0), pi / 2, r1, 0);
    denavit_hartenberg(m_2t, q(1) + pi / 2, pi, -r2t, 0);
    denavit_hartenberg(m_2 ,q(1) + pi / 2, pi, -r2, 0);
    denavit_hartenberg(m_3t, q(2), pi, -r3t, 0);
    denavit_hartenberg(m_3, q(2), pi, -r3, 0);

    // Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    q2rot(rotm, rotb(0), rotb(1), rotb(2));

    trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
             rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
             rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
             0, 0, 0, 1;

    // Position of the legs with respect to the base
    if (leg == 1) {
        reposition_leg(repos_leg, -3 * pi / 4, r4, -r4);
    }
    else if (leg == 2) {
        reposition_leg(repos_leg, -pi / 4, r4, r4);
    }
    else if (leg == 3) {
        reposition_leg(repos_leg, 3 * pi / 4, -r4, -r4);
    }
    else {
        reposition_leg(repos_leg, pi / 4, -r4, r4);
    }
    
    // Location of center of mass
    // Position and weight of the center of mass
    Matrix4d m_com1 = (w(0) / w(3)) * trans*repos_leg;
    Matrix4d m_com2 = (w(1) / w(3)) * trans*repos_leg*m_1*m_2t;
    Matrix4d m_com3 = (w(2) / w(3)) * trans*repos_leg*m_1*m_2*m_3t;
    Matrix4d mf = trans*repos_leg*m_1*m_2*m_3;

    // Position Vector
    Vector3d p1, p2, p3, p4;
    p1 << m_com1(0,3), m_com1(1,3), m_com1(2,3);
    p2 << m_com2(0,3), m_com2(1,3), m_com2(2,3);
    p3 << m_com3(0,3), m_com3(1,3), m_com3(2,3);
    p4 << mf(0,3), mf(1,3), mf(2,3);
    
    position << p1, p2, p3, p4;
}


void com_jacobian(MatrixXd& com_jacobian, const VectorXd& q, const Vector3d& current_pos) {
    // Analytic jacobian for leg position. Returns a 3x3 matrix with input q = [q1, q2, q3]
    VectorXd dq;
    Vector3d pos_dq;
    double delta = 0.00001;

    for (int i = 0; i <= 17; i++) {
        // Copy initial articular configuration and use delta increment on index i
        dq = q;
        dq(i) += delta;

        // Homegeneous Transformation Matrix after increment
        com_pos(pos_dq, dq);

        // Finite difference
        com_jacobian(0, i) = (pos_dq(0) - current_pos(0)) / delta;
        com_jacobian(1, i) = (pos_dq(1) - current_pos(1)) / delta;
        com_jacobian(2, i) = (pos_dq(2) - current_pos(2)) / delta;
    }
}


void full_kinematics(VectorXd& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg) {
    // This function finds the forward kinematics of each leg of the robot.
    // Returns the position of the end - effector according to the position and orientation of the base.This is possible
    // by calculating the jacobian of each leg a locating it in a homogeneous transformation matrix.

    Matrix3d rotm;
    Matrix4d leg_base, leg_str, leg_mid, leg_end, m_link1, m_link2, m_link3, trans, repos_leg;
    const double r1 = 5.5;     // Distance from servo 1 to 2
    const double r2 = 7.5;     // Distance from servo 2 to 3
    const double r3 = 22.5;    // Distance from servo 3 to effector
    const double r4 = 10.253;  // Distance from base to servo 1

    // Denavit - Hartenberg matrices
    denavit_hartenberg(m_link1, q(0), pi / 2, r1, 0);
    denavit_hartenberg(m_link2, q(1) + pi / 2, pi, -r2, 0);
    denavit_hartenberg(m_link3, q(2), pi, -r3, 0);

    // Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
    q2rot(rotm, rotb(0), rotb(1), rotb(2));

    trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
             rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
             rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
             0, 0, 0, 1;

    // Position of the legs with respect to the base
    if (leg == 1) {
        reposition_leg(repos_leg, -3 * pi / 4, r4, -r4);
    }
    else if (leg == 2) {
        reposition_leg(repos_leg, -pi / 4, r4, r4);
    }
    else if (leg == 3) {
        reposition_leg(repos_leg, 3 * pi / 4, -r4, -r4);
    }
    else {
        reposition_leg(repos_leg, pi / 4, -r4, r4);
    }

    // Returns the position of every link in the system
    leg_base = trans * repos_leg;
    leg_str = trans * repos_leg*m_link1;
    leg_mid = trans * repos_leg*m_link1*m_link2;
    leg_end = trans * repos_leg*m_link1*m_link2*m_link3;

    // Position vector[base, leg_start, leg_middle, leg_end]: (x, y, z)
    pos << leg_base(0, 3), leg_base(1, 3), leg_base(2, 3),
        leg_str(0, 3), leg_str(1, 3), leg_str(2, 3),
        leg_mid(0, 3), leg_mid(1, 3), leg_mid(2, 3),
        leg_end(0, 3), leg_end(1, 3), leg_end(2, 3);
}


void walking(MatrixXd& com_jacobian, const VectorXd& q, const Vector3d& current_pos, const Legs& Leg) {
    MatrixXd leg1, leg2, leg3, leg4, cog1, cog2;
    
    // Ingresamos el paso
    double x_dist, y_dist, steps;
    cout << endl << "Ingrese el desplazamiento en x:" << endl;
    cin >> x_dist;
    cout << endl << "Ingrese el desplazamiento en y:" << endl;
    cin >> y_dist;
    cout << endl << "Ingrese la secuencia:" << endl;
    cin >> steps;

    VectorXd desired_pos(10);

    // Increment leg1 position
    desired_pos(0) = Leg.leg1(0) + x_dist;
    desired_pos(1) = Leg.leg1(1) + y_dist;

    // Update center of gravity
    desired_pos(8) = Leg.com(0) + x_dist / 2;
    desired_pos(9) = Leg.com(1) + y_dist / 2;
        
    // Increment leg2 position
    desired_pos(2) = Leg.leg2(0) + x_dist;
    desired_pos(3) = Leg.leg2(1) + y_dist;

    // Increment leg4 position
    desired_pos(6) = Leg.leg4(0) + x_dist;
    desired_pos(7) = Leg.leg4(1) + y_dist;

    // Update center of gravity
    desired_pos(8) = Leg.com(0) + x_dist / 2;
    desired_pos(9) = Leg.com(1) + y_dist / 2;

    // Increment leg3 position
    desired_pos(4) = Leg.leg3(0) + x_dist;
    desired_pos(5) = Leg.leg3(1) + y_dist;
}


void compare(const VectorXd& xd, const Vector3d& com_xd, const Legs& pos) {
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " ", ";");
    cout << "Desired pos:" << xd.segment(3, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(6, 3).format(CommaInitFmt) << endl;
    cout << "            " << xd.segment(9, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(12, 3).format(CommaInitFmt) << endl;
    cout << "Final pos  :" << pos.leg1.format(CommaInitFmt) << "\n" << "            " << pos.leg2.format(CommaInitFmt) << endl;
    cout << "            " << pos.leg3.format(CommaInitFmt) << "\n" << "            " << pos.leg4.format(CommaInitFmt) << endl;
    cout << "Body pos   :" << xd.segment(0, 3).format(CommaInitFmt) << "\n" << "            " << pos.posb.format(CommaInitFmt) << endl;
    cout << "Body rot   :" << xd.segment(15, 3).format(CommaInitFmt) << "\n" << "            " << pos.rotb.format(CommaInitFmt) << endl;
    cout << "COM pos    :" << pos.com.format(CommaInitFmt) << "\n" << "            " << com_xd.format(CommaInitFmt) << endl << endl;
}