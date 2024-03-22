#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>

enum SolutionFlag{
	UNDEFINED, 
	SINGULARITY_1, // 2R arm is fully stretched.
	TARGET_ON_L1,
	SINGULARITY_2, // 2R arm is folded onto itself (target = origin).
	TWO_SOLUTIONS
};

SolutionFlag two_r_manip_ik(double x_target, double y_target, double L1, double L2, 
				   double sol[])
{
	SolutionFlag flag = UNDEFINED;
	double target_norm = pow(x_target, 2) + pow(y_target, 2);
	double c2 = (target_norm - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2);
	if (abs(c2) > 1)
	{	// Target is unreachable.
		flag = UNDEFINED;
	}
	else if (c2 == 1)
	{	// Target lies on circle with maximum reachable radius.
		sol[0] = atan2(y_target, x_target);
		sol[1] = 0;
		flag = SINGULARITY_1;
	}
	else if (c2 == -1 && !(target_norm == 0))
	{	// Target is within circle of radius abs(L1 - L2) centered at origin.
		sol[0] = atan2(y_target, x_target);
		sol[1] = M_PI;
		flag = TARGET_ON_L1;
	}
	else if (c2 == -1 && target_norm == 0)
	{	// Target is at the origin.
		flag = SINGULARITY_2;
	}
	else
	{	// Target is reachable and yields two joint solutions.
		flag = TWO_SOLUTIONS;
		double q21 = acos(c2);
		double q22 = -acos(c2);
		double theta = atan2(y_target, x_target);
		double q11 = theta - atan2(L2 * sin(q21), L1 + L2 * cos(q21));
		double q12 = theta - atan2(L2 * sin(q22), L1 + L2 * cos(q22));
		sol[0] = q11;
		sol[1] = q21;
		sol[2] = q12;
		sol[3] = q22;
	}
	
	return flag;
}

double compute_three_r_heading(double q2, double q3,
	double L1, double L2, double L3,
	double x_target, double y_target, bool direction)
{
	double gamma = atan((L2 * cos(q2) + L3 * cos(q2 + q3)) / L1);
	double theta = atan2(y_target, x_target);
	double q1 = gamma + theta;
	return q1;
}

SolutionFlag three_r_manip_ik(double x_target, double y_target, double z_target,
	                 double L1, double L2, double L3, 
	                 bool dir, double sol[])
{
	SolutionFlag flag = UNDEFINED;
	int x_2r = sqrt(pow(x_target, 2) + pow(y_target, 2) - pow(L1, 2));
	int y_2r = -z_target + L1;
	
	double sol_2r[4];

	// Solve 2RIK once, ignore alternate 3RIK solution that reaches backwards.
	SolutionFlag flag_2r = two_r_manip_ik(x_2r, y_2r, L2, L3, sol_2r);

	if (flag_2r == UNDEFINED || flag_2r == SINGULARITY_2)
	{
		flag = UNDEFINED;
	}
	else if (flag_2r == SINGULARITY_1 || flag_2r == TARGET_ON_L1)
	{
		double q2 = sol_2r[0];
		double q3 = sol_2r[1];
		double q1 = compute_three_r_heading(q2, q3,
			L1, L2, L3,
			x_target, y_target, dir);

		double joints[6] = { q1, q2, q3, 0, 0, 0 };
		std::copy(joints, joints + 6, sol);

		return UNDEFINED; // TODO
	}
	else
	{
		double q21 = sol_2r[0];
		double q31 = sol_2r[1];
		double q11 = compute_three_r_heading(q21, q31,
			L1, L2, L3,
			x_target, y_target, dir);

		double q22 = sol_2r[2];
		double q32 = sol_2r[3];
		double q12 = compute_three_r_heading(q22, q32,
			L1, L2, L3,
			x_target, y_target, dir);

		double joints[6] = { q11, q21, q31, q21, q22, q32 };
		std::copy(joints, joints + 6, sol);
		
		return UNDEFINED; // TODO
	}
	
	return flag;
}

int leg_ik()
{
	return 0;
}

int main(int argc, char* argv[])
{
	double res[4] = { 0, 0, 0, 0 };
	int flag = two_r_manip_ik(0, 0, 1, 1, res);

	std::cout << "Solution type: " << flag << std::endl;

	for (int i = 0; i < 4; i++)
	{
		std::cout << res[i] << std::endl;
	}
}