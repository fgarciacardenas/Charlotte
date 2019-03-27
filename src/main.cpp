#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <chrono>  // for high_resolution_clock

using namespace Eigen;
using namespace std;

// Declare structs
struct Cost_values
{
	MatrixXd h;
	VectorXd f;
};

struct QP_values
{
	MatrixXd qf;
	int itr;
};

void forward_kinematics(Vector3d& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg);

class Legs
{
	public:
	Vector3d leg1, leg2, leg3, leg4, posb, rotb;

	void update_position(const VectorXd& q)
	{
		posb = q.segment(0, 3);
		rotb = q.segment(15, 3);
		forward_kinematics(leg1, q.segment(3, 3), posb, rotb, 1);
		forward_kinematics(leg2, q.segment(6, 3), posb, rotb, 2);
		forward_kinematics(leg3, q.segment(9, 3), posb, rotb, 3);
		forward_kinematics(leg4, q.segment(12, 3), posb, rotb, 4);
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
	}
};

// Declare functions
void initialize(VectorXd& q);
void denavit_hartenberg(Matrix4d& mat, const double& theta, const double& alpha, const double& r, const double& d);
void reposition_leg(Matrix4d& leg_pos, const double& ang, const double& dx, const double& dy);
void q2rot(Matrix3d& rotm, const double& a, const double& b, const double& c);
void rot2q(Vector3d& values, const Matrix3d& rotm);
void jacobian_kinematics(Matrix3d& jacobian, const Vector3d& q, const Vector3d& rotb, const int& leg);
void quad_prog(QP_values& qp, VectorXd q, const VectorXd& xd, const Parameters& parameters, const Vector3d& com_xd, Legs& pos);
void costfunc(Cost_values& cost, const VectorXd& q, const VectorXd& xd, const int& lamb, const VectorXi& w, const VectorXd& com_xd, Legs& pos);
void leg_jacobian(MatrixXd& leg_jacobian, const Vector3d& q, const int& leg, Legs& pos);
void calc_err(double& err, const VectorXd& q, const VectorXd& xd, Legs& pos);
void com_pos(Vector3d& com_pos, const VectorXd& q);
void com_kinematics(VectorXd& position, const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb, const Vector4d& w);
void com_jacobian(Matrix3d& com_jacobian, const VectorXd& q, const Vector3d& posb, const Vector3d& rotb);
void full_kinematics(VectorXd& pos, const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg);
void compare(const VectorXd& xd, const Legs& pos);

// Constants
const double pi = M_PI;

int main() {
	VectorXd q(18), xd(18);
	Vector3d com_xd;
	QP_values qp;
	Legs pos;

	initialize(q);
	pos.update_position(q);

	// Desired position[posb, leg1, leg2, leg3, leg4, rotb]:
	xd.segment(0, 3)  << 3, 2, 0;
	xd.segment(3, 3)  << 2 + pos.leg1(0), 2 + pos.leg1(1), 0 + pos.leg1(2);
	xd.segment(6, 3)  << 0 + pos.leg2(0), 0 + pos.leg2(1), 0 + pos.leg2(2);
	xd.segment(9, 3)  << 0 + pos.leg3(0), 0 + pos.leg3(1), 0 + pos.leg3(2);
	xd.segment(12, 3) << 0 + pos.leg4(0), 0 + pos.leg4(1), 0 + pos.leg4(2);
	xd.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;

	// Desired position of COM
	//com_xd = cmass(q);
	com_xd << Vector3d::Zero();
	Parameters parameters;
	// Quadratic program
	auto start = std::chrono::high_resolution_clock::now();
	quad_prog(qp, q, xd, parameters, com_xd, pos);
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	std::cout << "Elapsed time qp: " << elapsed.count() << " s\n";
	// Compare desired and current positions(x, y, z)
	compare(xd, pos);
	// Find the center of mass
	//[com, __, __] = com_system(q)
}


void initialize(VectorXd& q) {
	// q : Initial configuration vector[posb, leg1, leg2, leg3, leg4, rotb]
	q.segment(0, 3) << 0, 0, 0;
	q.segment(3, 3) << 90 * pi/180, 90 * pi / 180, 90 * pi / 180;
	q.segment(6, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
	q.segment(9, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
	q.segment(12, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;
	q.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;
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

	rotm <<	rz*ry*rx;
}


void rot2q(Vector3d& values, const Matrix3d& rotm) {
	double a = atan2(rotm(2,1), rotm(2,2));
	double b = atan2(-rotm(2,0), sqrt(pow(rotm(2,1),2)) + (pow(rotm(2,2),2)));
	double c = atan2(rotm(1,0), rotm(0,0));
	values << a, b, c;
}


void jacobian_kinematics(Matrix3d& jacobian, const Vector3d& q, const Vector3d& rotb, const int& leg) {
	const double r1 = 5.5;     // Distance from servo 1 to 2
	const double r2 = 7.5;     // Distance from servo 2 to 3
	const double r3 = 22.5;    // Distance from servo 3 to effector
	const double r4 = 10.253;  // Distance from base to servo 1
	Matrix3d rotm;

	if (leg == 1) {
		jacobian << sin(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), -cos(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), r3 * cos(q(1) - q(2)) * cos(q(0) + pi / 4),
			        -cos(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), -sin(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), r3 * cos(q(1) - q(2)) * sin(q(0) + pi / 4),
			        0, r2 * sin(q(1)) + r3 * sin(q(1) - q(2)), -r3 * sin(q(1) - q(2));
	}
	else if (leg == 2) {
		jacobian << cos(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), sin(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), -r3 * cos(q(1) - q(2)) * sin(q(0) + pi / 4),
				    sin(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), -cos(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), r3 * cos(q(1) - q(2)) * cos(q(0) + pi / 4),
					0, r2 * sin(q(1)) + r3 * sin(q(1) - q(2)), -r3 * sin(q(1) - q(2));
	}
	else if (leg == 3) {
		jacobian << -cos(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), -sin(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), r3 * cos(q(1) - q(2)) * sin(q(0) + pi / 4),
				    -sin(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), cos(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), -r3 * cos(q(1) - q(2)) * cos(q(0) + pi / 4),
			        0, r2 * sin(q(1)) + r3 * sin(q(1) - q(2)), -r3 * sin(q(1) - q(2));
	}
	else {
		jacobian << -sin(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), cos(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), -r3 * cos(q(1) - q(2)) * cos(q(0) + pi / 4),
			cos(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), sin(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), -r3 * cos(q(1) - q(2)) * sin(q(0) + pi / 4),
			0, r2 * sin(q(1)) + r3 * sin(q(1) - q(2)), -r3 * sin(q(1) - q(2));
	}
	
	// Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
	q2rot(rotm, rotb(0), rotb(1), rotb(2));
	jacobian *= rotm;
}


void quad_prog(QP_values& qp, VectorXd q, const VectorXd& xd, const Parameters& parameters, const Vector3d& com_xd, Legs& pos) {
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
	MatrixXd qf(parameters.max_iter, 18);
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
		itr++;
		j++;
	}
	
	qp = { qf, itr };
}


void costfunc(Cost_values& cost, const VectorXd& q, const VectorXd& xd, const int& lamb, const VectorXi& w, const VectorXd& com_xd, Legs& pos) {
	// This function finds the values of h and f in order to initialize the quadratic program.
	// The inputs are q : actuated and sub - actuated angles, xd : desired position vector, p : weights and lamb : gain.

    // Position and orientation of the base
	Vector3d com;
	MatrixXd j1(3, 18), j2(3, 18), j3(3, 18), j4(3, 18), j5(3, 18), j6(3, 18), j_com;
	
	// Jacobian of each leg	
	leg_jacobian(j1, q.segment(3, 3), 1, pos);
	leg_jacobian(j2, q.segment(6, 3), 2, pos);
	leg_jacobian(j3, q.segment(9, 3), 3, pos);
	leg_jacobian(j4, q.segment(12, 3), 4, pos);

	// Jacobians of the base position and orientation
	j5 << Matrix3d::Identity(), Matrix<double, 3, 15>::Zero();
	j6 << Matrix<double, 3, 15>::Zero(), Matrix3d::Identity();

	// Position and jacobian of center of mass
	//com_pos(com, q);
	//com_jacobian(j_com, q, posb, rotb);

	// Values of h and f(Hessian and vector of linear elements) :
	MatrixXd h = w(0) * j1.transpose()*j1 + w(1) * j2.transpose()*j2 + w(2) * j3.transpose()*j3 + w(3) * j4.transpose()*j4 + w(4) * j5.transpose()*j5 + w(5) * j6.transpose()*j6;
	// + w(6) * j_com.transpose()*j_com
	
	VectorXd f = -2 * (w(1) * lamb * (pos.leg1 - xd.segment(3, 3)).transpose()*j1 + w(2) * lamb * (pos.leg2 - xd.segment(6, 3)).transpose()*j2 + w(3) * lamb * (pos.leg3 - xd.segment(9, 3)).transpose()*j3 + w(4) * lamb * (pos.leg4 - xd.segment(12, 3)).transpose()*j4 + w(0) * lamb * (pos.posb - xd.segment(0, 3)).transpose()*j5 + w(5) * lamb * (pos.rotb - xd.segment(15, 3)).transpose()*j6);
	// + w(6) * lamb * (com - com_xd).transpose()*j_com;

	cost = { h, f };
}


void leg_jacobian(MatrixXd& leg_jacobian, const Vector3d& q, const int& leg, Legs& pos) {
	Matrix3d jacobian, m_skew;

	jacobian_kinematics(jacobian, q, pos.rotb, leg);
	
	if (leg == 1) {
		// Distance vector of the end - effector with respect to base
		Vector3d d = pos.posb - pos.leg1;

		// Skew - Symmetric matrix
		m_skew << 0, -d(2), d(1),
			d(2), 0, -d(0),
			-d(1), d(0), 0;

		leg_jacobian << Matrix3d::Identity(), jacobian, Matrix<double, 3, 9>::Identity(), m_skew;
	}
	else if (leg == 2) {
		// Distance vector of the end - effector with respect to base
		Vector3d d = pos.posb - pos.leg2;

		// Skew - Symmetric matrix
		m_skew << 0, -d(2), d(1),
			d(2), 0, -d(0),
			-d(1), d(0), 0;

		leg_jacobian << Matrix3d::Identity(), Matrix3d::Zero(), jacobian, Matrix<double, 3, 6>::Identity(), m_skew;
	}
	else if (leg == 3) {
		// Distance vector of the end - effector with respect to base
		Vector3d d = pos.posb - pos.leg3;

		// Skew - Symmetric matrix
		m_skew << 0, -d(2), d(1),
			d(2), 0, -d(0),
			-d(1), d(0), 0;

		leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 6>::Identity(), jacobian, Matrix3d::Zero(), m_skew;
	}
	else {
		// Distance vector of the end - effector with respect to base
		Vector3d d = pos.posb - pos.leg4;

		// Skew - Symmetric matrix
		m_skew << 0, -d(2), d(1),
			d(2), 0, -d(0),
			-d(1), d(0), 0;

		leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 9>::Identity(), jacobian, m_skew;
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
	err = sqrt(pow(err_leg1(0), 2) + pow(err_leg1(1), 2) + pow(err_leg1(2), 2) + pow(err_leg2(0), 2) + pow(err_leg2(1), 2) + pow(err_leg2(2), 2) + pow(err_leg3(0), 2) + pow(err_leg3(1), 2) + pow(err_leg3(2), 2) + pow(err_leg4(0), 2) + pow(err_leg4(1), 2) + pow(err_leg4(2), 2) + pow(err_posb(0), 2) + pow(err_posb(1), 2) + pow(err_posb(2), 2) + pow(err_rotb(0), 2) + pow(err_rotb(1), 2) + pow(err_rotb(2), 2));
}


void com_pos(Vector3d& com_pos, const VectorXd& q) {
	// Weights
	double w_com1 = 0.075;
	double w_com2 = 0.15;
	double w_com3 = 0.2;
	double w_base = 0.7;
	double w_total = 4 * w_com1 + 4 * w_com2 + 4 * w_com3 + w_base;
	Vector3d posb, rotb;
	Vector4d w;
	VectorXd leg1_com(12), leg2_com(12), leg3_com(12), leg4_com(12);

	w << w_com1, w_com2, w_com3, w_total;
	posb << q.segment(0, 3);
	rotb << q.segment(15, 3);

	// Find the center of mass
	com_kinematics(leg1_com, q.segment(3, 3), 1, posb, rotb, w);
	com_kinematics(leg1_com, q.segment(6, 3), 2, posb, rotb, w);
	com_kinematics(leg1_com, q.segment(9, 3), 3, posb, rotb, w);
	com_kinematics(leg1_com, q.segment(12, 3), 4, posb, rotb, w);

	// COM of the base
	Vector3d base = (w_base / w_total) * posb;

	// COM position
	com_pos << leg1_com.segment(0, 3) + leg1_com.segment(3, 3) + leg1_com.segment(6, 3) + leg2_com.segment(0, 3) + leg2_com.segment(3, 3) + leg2_com.segment(6, 3) + leg3_com.segment(0, 3) + leg3_com.segment(3, 3) + leg3_com.segment(6, 3) + leg4_com.segment(0, 3) + leg4_com.segment(3, 3) + leg4_com.segment(6, 3) + base;
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


void com_jacobian(Matrix3d& com_jacobian, const VectorXd& q, const Vector3d& posb, const Vector3d& rotb) {
	Matrix3d jacobian1_com2, jacobian1_com3, jacobian2_com2, jacobian2_com3, jacobian3_com2, jacobian3_com3, jacobian4_com2, jacobian4_com3;
	
	// Variables
	const double r1 = 5.5;     // Distance from servo 1 to 2
	const double r2t = 3.75;   // Distance from servo 2 to com2
	const double r2 = 7.5;     // Distance from servo 2 to 3
	const double r3t = 11;     // Distance from servo 3 to com3
	const double r3 = 22.5;    // Distance from servo 3 to end - effector
	const double r4 = 10.253;  // Distance from base to servo 1

	// Weights
	const double w_com1 = 0.075;
	const double w_com2 = 0.15;
	const double w_com3 = 0.2;
	const double w_base = 0.7;
	const double w_total = 4 * w_com1 + 4 * w_com2 + 4 * w_com3 + w_base;

	// Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
	Matrix3d rotm;
	q2rot(rotm, rotb(0), rotb(1), rotb(2));

	Matrix4d trans;
	trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
			 rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
			 rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
			 0, 0, 0, 1;

	// Jacobian of center of mass
	jacobian1_com2 <<  sin(q(3) + pi / 4) * (r1 + r2t * sin(q(4))), -r2t * cos(q(4)) * cos(q(3) + pi / 4), 0,
					  -cos(q(3) + pi / 4) * (r1 + r2t * sin(q(4))), -r2t * cos(q(4)) * sin(q(3) + pi / 4), 0,
				       0, r2t * sin(q(4)), 0;

	jacobian1_com3 <<  sin(q(3) + pi / 4) * (r1 + r2 * sin(q(4)) + r3t * sin(q(4) - q(5))), -cos(q(3) + pi / 4) * (r2 * cos(q(4)) + r3t * cos(q(4) - q(5))), r3t * cos(q(4) - q(5)) * cos(q(3) + pi / 4),
					  -cos(q(3) + pi / 4) * (r1 + r2 * sin(q(4)) + r3t * sin(q(4) - q(5))), -sin(q(3) + pi / 4) * (r2 * cos(q(4)) + r3t * cos(q(4) - q(5))), r3t * cos(q(4) - q(5)) * sin(q(3) + pi / 4),
					   0, r2 * sin(q(4)) + r3t * sin(q(4) - q(5)), -r3t * sin(q(4) - q(5));

	jacobian2_com2 << cos(q(6) + pi / 4) * (r1 + r2t * sin(q(7))), r2t * cos(q(7)) * sin(q(6) + pi / 4), 0,
			   	      sin(q(6) + pi / 4) * (r1 + r2t * sin(q(7))), -r2t * cos(q(7)) * cos(q(6) + pi / 4), 0,
			          0, r2t * sin(q(7)), 0;

	jacobian2_com3 << cos(q(6) + pi / 4) * (r1 + r2 * sin(q(7)) + r3t * sin(q(7) - q(8))), sin(q(6) + pi / 4) * (r2 * cos(q(7)) + r3t * cos(q(7) - q(8))), -r3t * cos(q(7) - q(8)) * sin(q(6) + pi / 4),
					  sin(q(6) + pi / 4) * (r1 + r2 * sin(q(7)) + r3t * sin(q(7) - q(8))), -cos(q(6) + pi / 4) * (r2 * cos(q(7)) + r3t * cos(q(7) - q(8))), r3t * cos(q(7) - q(8)) * cos(q(6) + pi / 4),
					  0, r2 * sin(q(7)) + r3t * sin(q(7) - q(8)), -r3t * sin(q(7) - q(8));
	
	jacobian3_com2 << -cos(q(9) + pi / 4) * (r1 + r2t * sin(q(10))), -r2t * cos(q(10)) * sin(q(9) + pi / 4), 0,
					  -sin(q(9) + pi / 4) * (r1 + r2t * sin(q(10))), r2t * cos(q(10)) * cos(q(9) + pi / 4), 0,
			          0, r2t * sin(q(10)), 0;

	jacobian3_com3 << -cos(q(9) + pi / 4) * (r1 + r2 * sin(q(10)) + r3t * sin(q(10) - q(11))), -sin(q(9) + pi / 4) * (r2 * cos(q(10)) + r3t * cos(q(10) - q(11))), r3t * cos(q(10) - q(11)) * sin(q(9) + pi / 4),
					  -sin(q(9) + pi / 4) * (r1 + r2 * sin(q(10)) + r3t * sin(q(10) - q(11))), cos(q(9) + pi / 4) * (r2 * cos(q(10)) + r3t * cos(q(10) - q(11))), -r3t * cos(q(10) - q(11)) * cos(q(9) + pi / 4),
				      0, r2 * sin(q(10)) + r3t * sin(q(10) - q(11)), -r3t * sin(q(10) - q(11));
	
	jacobian4_com2 << -sin(q(12) + pi / 4) * (r1 + r2t * sin(q(13))), r2t * cos(q(13)) * cos(q(12) + pi / 4), 0,
				       cos(q(12) + pi / 4) * (r1 + r2t * sin(q(13))), r2t * cos(q(13)) * sin(q(12) + pi / 4), 0,
					   0, r2t * sin(q(13)), 0;

	jacobian4_com3 << -sin(q(12) + pi / 4) * (r1 + r2 * sin(q(13)) + r3t * sin(q(13) - q(14))), cos(q(12) + pi / 4) * (r2 * cos(q(13)) + r3t * cos(q(13) - q(14))), -r3t * cos(q(13) - q(14)) * cos(q(12) + pi / 4),
					   cos(q(12) + pi / 4) * (r1 + r2 * sin(q(13)) + r3t * sin(q(13) - q(14))), sin(q(12) + pi / 4) * (r2 * cos(q(13)) + r3t * cos(q(13) - q(14))), -r3t * cos(q(13) - q(14)) * sin(q(12) + pi / 4),
					   0, r2 * sin(q(13)) + r3t * sin(q(13) - q(14)), -r3t * sin(q(13) - q(14));
	
	// Reposition jacobian
	jacobian1_com2 = rotm * jacobian1_com2 * (w_com2 / w_total);
	jacobian1_com3 = rotm * jacobian1_com3 * (w_com3 / w_total);
	jacobian2_com2 = rotm * jacobian2_com2 * (w_com2 / w_total);
	jacobian2_com3 = rotm * jacobian2_com3 * (w_com3 / w_total);
	jacobian3_com2 = rotm * jacobian3_com2 * (w_com2 / w_total);
	jacobian3_com3 = rotm * jacobian3_com3 * (w_com3 / w_total);
	jacobian4_com2 = rotm * jacobian4_com2 * (w_com2 / w_total);
	jacobian4_com3 = rotm * jacobian4_com3 * (w_com3 / w_total);

	// COM jacobian
	com_jacobian = jacobian1_com2 + jacobian1_com3 + jacobian2_com2 + jacobian2_com3 + jacobian3_com2 + jacobian3_com3 + jacobian4_com2 + jacobian4_com3;
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

void compare(const VectorXd& xd, const Legs& pos) {
	IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " ", ";");
	cout << "Desired pos:" << xd.segment(3, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(6, 3).format(CommaInitFmt) << endl;
	cout << "            " << xd.segment(9, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(12, 3).format(CommaInitFmt) << endl;
	cout << "Final pos  :" << pos.leg1.format(CommaInitFmt) << "\n" << "            " << pos.leg2.format(CommaInitFmt) << endl;
	cout << "            " << pos.leg3.format(CommaInitFmt) << "\n" << "            " << pos.leg4.format(CommaInitFmt) << endl;
	cout << "Body pos   :" << xd.segment(0, 3).format(CommaInitFmt) << "\n" << "            " << pos.posb.format(CommaInitFmt) << endl;
	cout << "Body rot   :" << xd.segment(15, 3).format(CommaInitFmt) << "\n" << "            " << pos.rotb.format(CommaInitFmt) << endl;
}