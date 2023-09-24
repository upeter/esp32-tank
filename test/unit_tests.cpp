

#include <iostream>
#include <cmath>
#include <string.h>
using namespace std;
const double expo = 0.8;
const double nullFactor = 157;

double withExpo(double x)
{
	double n = x / nullFactor;
	double y = expo * pow(n, 3) + ((1.0 - expo) * n);
	return abs(y);
}

int main()
{
	// std::cout << "expo: " + to_string(pow(127)) << std::endl;
	int stickX = -127;
	std::cout << "expo: " + to_string((int)(stickX * withExpo(0.0 + stickX))) << std::endl;
	return 0;
}