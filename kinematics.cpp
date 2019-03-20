#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <qpOASES.hpp>

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
	int i;
};

// Declare functions
VectorXd initialize();
VectorXd forward_kinematics(const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg, const int& mode);
Matrix4d denavit_hartenberg(const double& theta, const double& alpha, const double& r, const double& d);
Matrix4d reposition_leg(const double& ang, const double& dx, const double& dy);
Matrix3d q2rot(const double& a, const double& b, const double& c);
MatrixXd leg_jacobian(const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb);
Cost_values costfunc(const VectorXd& q, const VectorXd& xd, const int& lamb, const VectorXi& w, const VectorXd& com_xd);
Matrix3d jacobian_kinematics(const Vector3d& q, const Vector3d& rotb, const int& leg);
MatrixXd com_pos(const VectorXd& q);
VectorXd com_kinematics(const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb, const Vector4d& w);
Matrix3d com_jacobian(const VectorXd& q, const Vector3d& posb, const Vector3d& rotb);
Vector3d rot2q(const Matrix3d& rotm);
QP_values quad_prog(VectorXd q, const VectorXd& xd, const int& max_iter, const double& d_t, const int& lamb, const VectorXi& w, const double& tol, const Vector3d& com_xd);
void compare(const MatrixXd& qf, const VectorXd& xd, const int& i);
double calc_err(const VectorXd& q, const VectorXd& xd);

// Constants
const double pi = M_PI;

int main()
{
	VectorXd q = initialize();
	Vector3d posb, rotb;
	posb << q.segment(0, 3);
	rotb << q.segment(15, 3);

	// Forward kinematics[leg1, leg2, leg3, leg4]:
	Vector3d leg1_position = forward_kinematics(q.segment(3, 3), posb, rotb, 1, 1);
	Vector3d leg2_position = forward_kinematics(q.segment(6, 3), posb, rotb, 2, 1);
	Vector3d leg3_position = forward_kinematics(q.segment(9, 3), posb, rotb, 3, 1);
	Vector3d leg4_position = forward_kinematics(q.segment(12, 3), posb, rotb, 4, 1);

	// Desired position[posb, leg1, leg2, leg3, leg4, rotb]:
	VectorXd xd(18);
	xd.segment(0, 3)  << 3, 2, 0;
	xd.segment(3, 3)  << 2 + leg1_position(0), 2 + leg1_position(1), 0 + leg1_position(2);
	xd.segment(6, 3)  << 0 + leg2_position(0), 0 + leg2_position(1), 0 + leg2_position(2);
	xd.segment(9, 3)  << 0 + leg3_position(0), 0 + leg3_position(1), 0 + leg3_position(2);
	xd.segment(12, 3) << 0 + leg4_position(0), 0 + leg4_position(1), 0 + leg4_position(2);
	xd.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;;

	// Desired position of COM
	//com_xd = cmass(q);
	Vector3d com_xd = Vector3d::Zero();
	// Maximum number of iterations
	const int max_iter = 30;
	// Time between signals
	const double d_t = 0.01;
	// Gain of quadratic function
	const int lamb = -10;
	// Maximum tolerance for minimization
	const double tol = 0.001;
	// Weights: [posb, leg1, leg2, leg3, leg4, rotb]
	VectorXi w = VectorXi::Ones(7);
	// Quadratic program
	QP_values qp = quad_prog(q, xd, max_iter, d_t, lamb, w, tol, com_xd);
	// Compare desired and current positions(x, y, z)
	compare(qp.qf, xd, qp.i);
	// Find the center of mass
	//[com, __, __] = com_system(q)

	std::cin.get();
}


VectorXd initialize() {
	// q : Initial configuration vector[posb, leg1, leg2, leg3, leg4, rotb]
	VectorXd q(18);
	q.segment(0, 3) << 0, 0, 0;
	q.segment(3, 3) << 90 * pi/180, 90 * pi / 180, 90 * pi / 180;
	q.segment(6, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;;
	q.segment(9, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;;
	q.segment(12, 3) << 90 * pi / 180, 90 * pi / 180, 90 * pi / 180;;
	q.segment(15, 3) << 0 * pi / 180, 0 * pi / 180, 0 * pi / 180;;
	return q;
}


VectorXd forward_kinematics(const Vector3d& q, const Vector3d& posb, const Vector3d& rotb, const int& leg, const int& mode) {
	// This function finds the forward kinematics of each leg of the robot.
	// Returns the position of the end - effector according to the position and orientation of the base.This is possible
	// by calculating the jacobian of each leg a locating it in a homogeneous transformation matrix.
	
	Matrix4d leg_end;
	double r1 = 5.5;     // Distance from servo 1 to 2
	double r2 = 7.5;     // Distance from servo 2 to 3
	double r3 = 22.5;    // Distance from servo 3 to effector
	double r4 = 10.253;  // Distance from base to servo 1

	// Denavit - Hartenberg matrices
	Matrix4d m_link1 = denavit_hartenberg(q(0), pi / 2, r1, 0);
	Matrix4d m_link2 = denavit_hartenberg(q(1) + pi / 2, pi, -r2, 0);
	Matrix4d m_link3 = denavit_hartenberg(q(2), pi, -r3, 0);

	// Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
	Matrix3d rotm = q2rot(rotb(0), rotb(1), rotb(2));

	Matrix4d trans;
	trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
		     rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
		     rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
		     0, 0, 0, 1;

	// Position of the legs with respect to the base
	Matrix4d repos_leg;
	if (leg == 1) {
		repos_leg << reposition_leg(-3 * pi / 4, r4, -r4);
	}
	else if (leg == 2) {
		repos_leg << reposition_leg(-pi / 4, r4, r4);
	}
	else if (leg == 3) {
		repos_leg << reposition_leg(3 * pi / 4, -r4, -r4);
	}
	else if (leg == 4) {
		repos_leg << reposition_leg(pi / 4, -r4, r4);
	}

	// Mode 1 returns only the position of the end - effector
	if (mode == 1) {
		leg_end << trans*repos_leg*m_link1*m_link2*m_link3;
		
		// End - effector position(x, y, z)
		VectorXd pos(3);
		pos << leg_end(0, 3), leg_end(1, 3), leg_end(2, 3);
		return pos;
	}

	// Mode 2 returns the position of every link in the system
	else if (mode == 2) {
		Matrix4d leg_base, leg_str, leg_mid, leg_end;
		leg_base = trans * repos_leg;
		leg_str = trans * repos_leg*m_link1;
		leg_mid = trans * repos_leg*m_link1*m_link2;
		leg_end = trans * repos_leg*m_link1*m_link2*m_link3;

		// Position vector[base, leg_start, leg_middle, leg_end]: (x, y, z)
		VectorXd pos(12);
		pos << leg_base(0, 3), leg_base(1, 3), leg_base(2, 3), 
			    leg_str(0, 3),  leg_str(1, 3),  leg_str(2, 3),
			    leg_mid(0, 3),  leg_mid(1, 3),  leg_mid(2, 3),
			    leg_end(0, 3),  leg_end(1, 3),  leg_end(2, 3);

		return pos;
	}
}


Matrix4d denavit_hartenberg(const double& theta, const double& alpha, const double& r, const double& d) {
	Matrix4d mat;
	mat << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), r * cos(theta),
		   sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta),
		   0.0, sin(alpha), cos(alpha), d,
		   0.0, 0.0, 0.0, 1.0;
	return mat;
}


Matrix4d reposition_leg(const double& ang, const double& dx, const double& dy) {
	Matrix4d leg_pos;
	leg_pos << cos(ang), -sin(ang), 0, dx,
			   sin(ang),  cos(ang), 0, dy,
			   0, 0, 1, 0,
			   0, 0, 0, 1;
	return leg_pos;
}


Matrix3d q2rot(const double& a, const double& b, const double& c) {
	Matrix3d rx, ry, rz, rotm;
	
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
	return rotm;
}


Vector3d rot2q(const Matrix3d& rotm) {
	double a, b, c;
	Vector3d values;
	a = atan2(rotm(2,1), rotm(2,2));
	b = atan2(-rotm(2,0), sqrt(pow(rotm(2,1),2)) + (pow(rotm(2,2),2)));
	c = atan2(rotm(1,0), rotm(0,0));
	values << a, b, c;
	return values;
}


Matrix3d jacobian_kinematics(const Vector3d& q, const Vector3d& rotb, const int& leg) {
	Matrix3d jacobian;
	const double r1 = 5.5;     // Distance from servo 1 to 2
	const double r2 = 7.5;     // Distance from servo 2 to 3
	const double r3 = 22.5;    // Distance from servo 3 to effector
	const double r4 = 10.253;  // Distance from base to servo 1

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
	else if (leg == 4) {
		jacobian << -sin(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), cos(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), -r3 * cos(q(1) - q(2)) * cos(q(0) + pi / 4),
			cos(q(0) + pi / 4) * (r1 + r2 * sin(q(1)) + r3 * sin(q(1) - q(2))), sin(q(0) + pi / 4) * (r2 * cos(q(1)) + r3 * cos(q(1) - q(2))), -r3 * cos(q(1) - q(2)) * sin(q(0) + pi / 4),
			0, r2 * sin(q(1)) + r3 * sin(q(1) - q(2)), -r3 * sin(q(1) - q(2));
	}
	
	// Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
	Matrix3d rotm = q2rot(rotb(0), rotb(1), rotb(2));

	jacobian = rotm * jacobian;
	return jacobian;
}


QP_values quad_prog(VectorXd q, const VectorXd& xd, const int& max_iter, const double& d_t, const int& lamb, const VectorXi& w, const double& tol, const Vector3d& com_xd) {
	// This function manages the minimization program and find the error of the desired function
	MatrixXd qf(max_iter, 18);
	int itr = 0;
	int j = 0;

	while (itr < max_iter) {
		// Quadratic programming
		Cost_values cost = costfunc(q, xd, lamb, w, com_xd);
		MatrixXd h = (cost.h + cost.h.transpose()) / 2;
		VectorXd f = cost.f;

		double arr_h[324];
		Map<MatrixXd>(&arr_h[0], 18, 18) = h;

		double arr_f[18];
		Map<VectorXd>(&arr_f[0], 18, 1) = f;

		USING_NAMESPACE_QPOASES

		/* Setup data of first QP. */
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

		/* Setting up QProblemB object. */
		QProblemB qp_quad(18);

		Options options;
		//options.enableFlippingBounds = BT_FALSE;
		options.initialStatusBounds = ST_INACTIVE;
		options.numRefinementSteps = 1;
		options.enableCholeskyRefactorisation = 1;
		options.printLevel = PL_NONE;
		qp_quad.setOptions(options);

		/* Solve first QP. */
		int_t nWSR = 10;
		qp_quad.init(H, g, lb, ub, nWSR, 0);

		/* Get and print solution of first QP. */
		real_t xOpt[18];
		VectorXd dq(18);
		qp_quad.getPrimalSolution(xOpt);
		for (int i = 0; i < 18; ++i) {
			dq[i] = xOpt[i];
		}
		//printf("\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt, qp_quad.getObjVal());

		/* Solver configuration
		solvers.options['show_progress'] = False;
		solvers.options['maxiters'] = 1000;
		solvers.options['abstol'] = 1e-12;
		solvers.options['reltol'] = 1e-12;
		solvers.options['feastol'] = 1e-100;
		solvers.options['xtol'] = 1e-12;
		sol = solvers.qp(h, f);
		dq = array(sol['x']); */

		// Update the position vector
		q.segment(0, 15) = q.segment(0, 15) + d_t * dq.segment(0, 15);

		// Update the orientation vector
		Matrix3d rot_axis, skw, rgs, rot;
		rot_axis << cos(q(16)) * cos(q(17)), -sin(q(17)), 0,
					cos(q(16)) * sin(q(17)),  cos(q(17)), 0,
				                -sin(q(16)),           0, 1;

		Vector3d rot_vec;
		rot_vec << rot_axis * dq.segment(15, 3);
		
		skw <<           0, -rot_vec(2),  rot_vec(1),
			    rot_vec(2),           0, -rot_vec(0),
			   -rot_vec(1),  rot_vec(0),           0;

		// Rodrigues rotation formula
		rgs = Matrix3d::Identity() + sin(d_t) * skw + (1 - cos(d_t)) * (skw*skw);
		rot = q2rot(q(15), q(16), q(17));
		rot *= rgs;
		q.segment(15, 3) = rot2q(rot);

		for (int i = 0; i < 18; ++i) {
			qf(j, i) = q(i);
		}

		double err = calc_err(q, xd);
		if (err <= tol) {
			itr++;
			break;
		}

		itr++;
		j++;
	}
	
	QP_values result = { qf, itr };

	return result;
}


Cost_values costfunc(const VectorXd& q, const VectorXd& xd, const int& lamb, const VectorXi& w, const VectorXd& com_xd) {
	// This function finds the values of h and f in order to initialize the quadratic program.
	// The inputs are q : actuated and sub - actuated angles, xd : desired position vector, p : weights and lamb : gain.

    // Position and orientation of the base
	Vector3d posb, rotb;
	posb << q.segment(0, 3);
	rotb << q.segment(15, 3);

	// Position and jacobian of each leg	
	Vector3d pos1 = forward_kinematics(q.segment(3, 3), posb, rotb, 1, 1);
	Vector3d pos2 = forward_kinematics(q.segment(6, 3), posb, rotb, 2, 1);
	Vector3d pos3 = forward_kinematics(q.segment(9, 3), posb, rotb, 3, 1);
	Vector3d pos4 = forward_kinematics(q.segment(12, 3), posb, rotb, 4, 1);
	
	MatrixXd j1 = leg_jacobian(q.segment(3, 3), 1, posb, rotb);
	MatrixXd j2 = leg_jacobian(q.segment(6, 3), 2, posb, rotb);
	MatrixXd j3 = leg_jacobian(q.segment(9, 3), 3, posb, rotb);
	MatrixXd j4 = leg_jacobian(q.segment(12, 3), 4, posb, rotb);

	// Jacobians of the base position and orientation
	MatrixXd j5(3,18), j6(3,18);
	j5 << Matrix3d::Identity(), Matrix<double, 3, 15>::Zero();
	j6 << Matrix<double, 3, 15>::Zero(), Matrix3d::Identity();

	// Position and jacobian of center of mass
	Vector3d com = com_pos(q);
	Matrix3d j_com = com_jacobian(q, posb, rotb);

	// Values of h and f(Hessian and vector of linear elements) :
	MatrixXd h = w(0) * j1.transpose()*j1 + w(1) * j2.transpose()*j2 + w(2) * j3.transpose()*j3 + w(3) * j4.transpose()*j4 + w(4) * j5.transpose()*j5 + w(5) * j6.transpose()*j6;
	// + w(6) * j_com.transpose()*j_com
	
	VectorXd f = -2 * (w(1) * lamb * (pos1 - xd.segment(3, 3)).transpose()*j1 + w(2) * lamb * (pos2 - xd.segment(6, 3)).transpose()*j2 + w(3) * lamb * (pos3 - xd.segment(9, 3)).transpose()*j3 + w(4) * lamb * (pos4 - xd.segment(12, 3)).transpose()*j4 + w(0) * lamb * (posb - xd.segment(0, 3)).transpose()*j5 + w(5) * lamb * (rotb - xd.segment(15, 3)).transpose()*j6);
	// + w(6) * lamb * (com - com_xd).transpose()*j_com;

	Cost_values result = { h, f };

	return result;
}


MatrixXd leg_jacobian(const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb) {
	Vector3d pos = forward_kinematics(q, posb, rotb, leg, 1);
	Matrix3d jacobian = jacobian_kinematics(q, rotb, leg);
	// Distance vector of the end - effector with respect to base
	Vector3d d = posb - pos;
	// Skew - Symmetric matrix
	Matrix3d m_skew;
	m_skew <<     0, -d(2),  d(1),
			   d(2),     0, -d(0),
			  -d(1),  d(0),     0;

	MatrixXd leg_jacobian(3,18);
	if (leg == 1) {
		leg_jacobian << Matrix3d::Identity(), jacobian, Matrix<double, 3, 9>::Identity(), m_skew;
	}
	else if (leg == 2) {
		leg_jacobian << Matrix3d::Identity(), Matrix3d::Zero(), jacobian, Matrix<double, 3, 6>::Identity(), m_skew;
	}
	else if (leg == 3) {
		leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 6>::Identity(), jacobian, Matrix3d::Zero(), m_skew;
	}
	else if (leg == 4) {
		leg_jacobian << Matrix3d::Identity(), Matrix<double, 3, 9>::Identity(), jacobian, m_skew;
	}

	return leg_jacobian;
}


double calc_err(const VectorXd& q, const VectorXd& xd) {
	// Find the current position
	Vector3d posb, rotb;
	posb << q.segment(0, 3);
	rotb << q.segment(15, 3);

	// Forward kinematics[leg1, leg2, leg3, leg4]:
	Vector3d leg1_position = forward_kinematics(q.segment(3, 3), posb, rotb, 1, 1);
	Vector3d leg2_position = forward_kinematics(q.segment(6, 3), posb, rotb, 2, 1);
	Vector3d leg3_position = forward_kinematics(q.segment(9, 3), posb, rotb, 3, 1);
	Vector3d leg4_position = forward_kinematics(q.segment(12, 3), posb, rotb, 4, 1);

	// Find the error of each leg and the base
	Vector3d err_leg1 = xd.segment(3, 3) - leg1_position;
	Vector3d err_leg2 = xd.segment(6, 3) - leg2_position;
	Vector3d err_leg3 = xd.segment(9, 3) - leg3_position;
	Vector3d err_leg4 = xd.segment(12, 3) - leg4_position;
	Vector3d err_posb = xd.segment(0, 3) - posb;
	Vector3d err_rotb = xd.segment(15, 3) - rotb;

	// Sum of the squared errors
	double err = sqrt(pow(err_leg1(0), 2) + pow(err_leg1(1), 2) + pow(err_leg1(2), 2) + pow(err_leg2(0), 2) + pow(err_leg2(1), 2) + pow(err_leg2(2), 2) + pow(err_leg3(0), 2) + pow(err_leg3(1), 2) + pow(err_leg3(2), 2) + pow(err_leg4(0), 2) + pow(err_leg4(1), 2) + pow(err_leg4(2), 2) + pow(err_posb(0), 2) + pow(err_posb(1), 2) + pow(err_posb(2), 2) + pow(err_rotb(0), 2) + pow(err_rotb(1), 2) + pow(err_rotb(2), 2));
	return err;
}


MatrixXd com_pos(const VectorXd& q) {
	// Weights
	double w_com1 = 0.075;
	double w_com2 = 0.15;
	double w_com3 = 0.2;
	double w_base = 0.7;
	double w_total = 4 * w_com1 + 4 * w_com2 + 4 * w_com3 + w_base;
	Vector4d w;
	w << w_com1, w_com2, w_com3, w_total;

	Vector3d posb, rotb;
	posb << q.segment(0, 3);
	rotb << q.segment(15, 3);

	// Find the center of mass
	VectorXd leg1_com = com_kinematics( q.segment(3, 3), 1, posb, rotb, w);
	VectorXd leg2_com = com_kinematics( q.segment(6, 3), 2, posb, rotb, w);
	VectorXd leg3_com = com_kinematics( q.segment(9, 3), 3, posb, rotb, w);
	VectorXd leg4_com = com_kinematics(q.segment(12, 3), 4, posb, rotb, w);

	// COM of the base
	Vector3d base = (w_base / w_total) * Vector3d::Zero();

	// COM position
	Vector3d com_pos;
	com_pos << leg1_com.segment(0, 3) + leg1_com.segment(3, 3) + leg1_com.segment(6, 3) + leg2_com.segment(0, 3) + leg2_com.segment(3, 3) + leg2_com.segment(6, 3) + leg3_com.segment(0, 3) + leg3_com.segment(3, 3) + leg3_com.segment(6, 3) + leg4_com.segment(0, 3) + leg4_com.segment(3, 3) + leg4_com.segment(6, 3) + base;
	return com_pos;
}


VectorXd com_kinematics(const Vector3d& q, const int& leg, const Vector3d& posb, const Vector3d& rotb, const Vector4d& w) {
	// Variables
	const double r1 = 5.5;     // Distance from servo 1 to 2
	const double r2t = 3.75;   // Distance from servo 2 to com2
	const double r2 = 7.5;     // Distance from servo 2 to 3
	const double r3t = 11;     // Distance from servo 3 to com3
	const double r3 = 22.5;    // Distance from servo 3 to end - effector
	const double r4 = 10.253;  // Distance from base to servo 1

	// Weights
	const double w_com1 = w(0);
	const double w_com2 = w(1);
	const double w_com3 = w(2);
	const double w_total = w(3);

	// Denavit - Hartenberg matrices
	Matrix4d m_1  = denavit_hartenberg(q(0), pi / 2, r1, 0);
	Matrix4d m_2t = denavit_hartenberg(q(1) + pi / 2, pi, -r2t, 0);
	Matrix4d m_2  = denavit_hartenberg(q(1) + pi / 2, pi, -r2, 0);
	Matrix4d m_3t = denavit_hartenberg(q(2), pi, -r3t, 0);
	Matrix4d m_3  = denavit_hartenberg(q(2), pi, -r3, 0);

	// Homogeneous Transformation Matrix - Reposition in respect to position and orientation of the base
	Matrix3d rotm = q2rot(rotb(0), rotb(1), rotb(2));

	Matrix4d trans;
	trans << rotm(0, 0), rotm(0, 1), rotm(0, 2), posb(0),
			 rotm(1, 0), rotm(1, 1), rotm(1, 2), posb(1),
			 rotm(2, 0), rotm(2, 1), rotm(2, 2), posb(2),
			 0, 0, 0, 1;

	// Position of the legs with respect to the base
	Matrix4d repos_leg;
	if (leg == 1) {
		repos_leg << reposition_leg(-3 * pi / 4, r4, -r4);
	}
	else if (leg == 2) {
		repos_leg << reposition_leg(-pi / 4, r4, r4);
	}
	else if (leg == 3) {
		repos_leg << reposition_leg(3 * pi / 4, -r4, -r4);
	}
	else if (leg == 4) {
		repos_leg << reposition_leg(pi / 4, -r4, r4);
	}
	
	// Location of center of mass
	// Position and weight of the center of mass
	Matrix4d m_com1 = (w_com1 / w_total) * trans*repos_leg;
	Matrix4d m_com2 = (w_com2 / w_total) * trans*repos_leg*m_1*m_2t;
	Matrix4d m_com3 = (w_com3 / w_total) * trans*repos_leg*m_1*m_2*m_3t;
	Matrix4d mf = trans*repos_leg*m_1*m_2*m_3;

	// Position Vector
	Vector3d p1, p2, p3, p4;
	p1 << m_com1(0,3), m_com1(1,3), m_com1(2,3);
	p2 << m_com2(0,3), m_com2(1,3), m_com2(2,3);
	p3 << m_com3(0,3), m_com3(1,3), m_com3(2,3);
	p4 << mf(0,3), mf(1,3), mf(2,3);
	VectorXd position(12);
	position << p1, p2, p3, p4;

	return position;
}


Matrix3d com_jacobian(const VectorXd& q, const Vector3d& posb, const Vector3d& rotb) {
	Matrix3d jacobian1_com2, jacobian1_com3, jacobian2_com2, jacobian2_com3, jacobian3_com2, jacobian3_com3, jacobian4_com2, jacobian4_com3, com_jacobian;
	
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
	Matrix3d rotm = q2rot(rotb(0), rotb(1), rotb(2));

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

	return com_jacobian;
}


void compare(const MatrixXd& qf, const VectorXd& xd, const int& i) {
	// Final position
	Vector3d posb, rotb;
	posb <<  qf(i - 1, 0),  qf(i - 1, 1),  qf(i - 1, 2);
	rotb << qf(i - 1, 15), qf(i - 1, 16), qf(i - 1, 17);
	
	Vector3d leg1, leg2, leg3, leg4;
	leg1 <<  qf(i - 1, 3),  qf(i - 1, 4),  qf(i - 1, 5);
	leg2 <<  qf(i - 1, 6),  qf(i - 1, 7),  qf(i - 1, 8);
	leg3 <<  qf(i - 1, 9), qf(i - 1, 10), qf(i - 1, 11);
	leg4 << qf(i - 1, 12), qf(i - 1, 13), qf(i - 1, 14);

	// Forward kinematics[leg1, leg2, leg3, leg4]:
	Vector3d leg1_position = forward_kinematics(leg1, posb, rotb, 1, 1);
	Vector3d leg2_position = forward_kinematics(leg2, posb, rotb, 2, 1);
	Vector3d leg3_position = forward_kinematics(leg3, posb, rotb, 3, 1);
	Vector3d leg4_position = forward_kinematics(leg4, posb, rotb, 4, 1);

	IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " ", ";");
	cout << "Desired pos:" << xd.segment(3, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(6, 3).format(CommaInitFmt) << endl;
	cout << "            " << xd.segment(9, 3).format(CommaInitFmt) << "\n" << "            " << xd.segment(12, 3).format(CommaInitFmt) << endl;
	cout << "Final pos  :" << leg1_position.format(CommaInitFmt) << "\n" << "            " << leg2_position.format(CommaInitFmt) << endl;
	cout << "            " << leg3_position.format(CommaInitFmt) << "\n" << "            " << leg4_position.format(CommaInitFmt) << endl;
	cout << "Body pos   :" << xd.segment(0, 3).format(CommaInitFmt) << "\n" << "            " << posb.format(CommaInitFmt) << endl;
	cout << "Body rot   :" << xd.segment(15, 3).format(CommaInitFmt) << "\n" << "            " << rotb.format(CommaInitFmt) << endl;
}
