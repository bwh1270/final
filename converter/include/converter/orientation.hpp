#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <iostream>
#include <cmath>
using namespace std;

struct Quaternion {
	float w, x, y, z;
};
struct EulerAngles {
	float roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q);
Quaternion ToQuaternion(EulerAngles angles);

#endif
