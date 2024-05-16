#include <math.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

std::vector<std::array<double,4>> super_fib_list(int n)
{
	std::vector<std::array<double,4>> Q(n);

	double dn = 1.0 / (double)n;
	double mc0 = 1.0 / sqrt(2.0);
	double mc1 = 1.0 / 1.533751168755204288118041;

	for (int i = 0; i < n; i++)
	{
		double s = (double)i+0.5;
		double ab = 2.0 * M_PI * s;
		double alpha = ab * mc0;
		double beta = ab * mc1;
		s *= dn;
		double r = sqrt(s);
		double R = sqrt(1.0-s);
		Q[i][0] = r*sin(alpha);
		Q[i][1] = r*cos(alpha);
		Q[i][2] = R*sin(beta);
		Q[i][3] = R*cos(beta);
	}
	return Q;
}

extern "C" {
        void super_fib_list_c(double* output, int n) {
                // Call the C++ function
                std::vector<std::array<double, 4>> result = super_fib_list(n);

                // Copy the result to the C array
                for (int i = 0; i < n; i++)
                {
                        output[4*i + 0] = result[i][0];
                        output[4*i + 1] = result[i][1];
                        output[4*i + 2] = result[i][2];
                        output[4*i + 3] = result[i][3];
                }
        }
}

