#include "newton_raphson.h"

// f(r) = (1 + k1*r² + k2*r⁴)r for which we want to find the root
double func(double r, double k1, double k2, double r_d)
{
    return r + k1 * pow(r, 3) + k2 * pow(r, 5) - r_d;
}

// Derivative of the above function f(r)
double derivFunc(double r, double k1, double k2)
{
    return 1 + 3 * k1 * pow(r, 2) + 5 * k2 * pow(r, 4);
}

// Function to find the root of f(r)
// r_d: the distorted normalized radius
// k1, k2: distortion coefficients
double newtonRaphson(double r_d, double k1, double k2)
{
    double r = r_d; // initial guess
    double h = func(r, k1, k2, r_d) / derivFunc(r, k1, k2);
    int nr_iterations = 0;

    while (fabs(h) >= EPSILON && nr_iterations < 1000)
    {
        h = func(r, k1, k2, r_d) / derivFunc(r, k1, k2);

        // r(i+1) = r(i) - f(x) / f'(x)
        r = r - h;
        nr_iterations = nr_iterations + 1;
    }

    return r;
}