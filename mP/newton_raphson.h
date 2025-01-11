#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define EPSILON 0.0000001

double func(double r, double k1, double k2, double r_d);
double derivFunc(double r, double k1, double k2);
double newtonRaphson(double r0, double k1, double k2);