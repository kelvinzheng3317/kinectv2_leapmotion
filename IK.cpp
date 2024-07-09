#include <iostream>
#include <vector>
#include <cmath>
#include <math.h>
#include <windows.h>
#include "IK.h"
// from my_teachMover import TeachMover

using namespace std;

#define PI 3.14159265

// initialize robot current coordinates
IK::IK(float x = 7, float y = 0, float z = 14.625) {
    this->x = x;
    this->y = y;
    this->z = z;
    return;
}

// Uses the robot's current coordinates to determine what the new stepper values after given change
// FIXME: add the ability to control gripper angle based off palm vector
int IK::FindStep(float newX, float newY, float newZ, float directionChange = 0) {
    x = newX;
    y = newY;
    z = newZ;
    cout << "Robot target coordinates - x: " << x << ", y: " << y << ", z: " << z << endl;
    float z0 = z - H;
    float Lxy = sqrt(pow(x, 2) + pow(y, 2));
    float l1 = sqrt(pow(Lxy, 2) + pow(z0, 2)) / 2;
    // print(f"z0: {z0}, Lxy: {Lxy}, l1: {l1}")

    try {
        float phi1 = acos(l1 / L);
        float phi3 = atan(z0 / Lxy);
        float theta1 = atan(y / x); // determines the step for the Base motor
        float theta2 = phi1 + phi3; // determines step for the Shoulder motor
        float theta3 = phi3 - phi1; // determines step for the Elbow motor
        // print(f"phi1: {math.degrees(phi1)}, phi3: {math.degrees(phi3)}, theta1: {math.degrees(theta1)}, theta2: {math.degrees(theta2)}, theta3: {math.degrees(theta3)}")
        int step1 = int(theta1 * B_C) + 1768;
        int step2 = int((PI/2 - theta2) * S_C) + 1100;
        int step3 = int((PI/2 - theta3) * E_C);
        cout << "IK results - step1: " << step1 << ", step2: " << step2 << ", step3 : " << step3 << endl;
        return step1, step2, step3, 420, 0; // returning 0's for step4 and step5 so number of return values matches Zilin's IK implementation, makes switch btw implementations easier
    }
    catch (...) {
        cout << "Math Domain Error" << endl;
        return -1, -1, -1, -1, -1;
    }


}

float IK::getCoords() {
    return x, y, z;
}

// update robots coordinates
void IK::incrCoords(float x, float y, float z) {
    this->x += x;
    this->y += y;
    this->z += z;
}
};

int main(int argc, char* argv[]) {
    IK IK();

    // NOTE : Testing IK and TeachMover in combination
    TeachMover robot('COM3');
    // the current steps for each motor must be initalized, this is my guesses for what the default position motors are
    // robot.set_motor_vals(1768, 1100, 1040, 0, 0, 0)
    robot.print_motors();

    int j1, j2, j3, j4, j5 = IK.FindStep(8, 0, 20);
    //# print(f"target motor steps: {j1} {j2} {j3} {j4} {j5}") # Default steps is 3536, 3536, 2079
    robot.set_step(240, j1, j2, j3, j4, j5, 750);
    sleep(3);
    robot.lock_wait();
    robot.returnToStart();

    // NOTE: Testing ranges for dx, dy, dz
    // dx, dy, dz = -7, -7, -7
    // while dx < 7:
    //     while dy < 7:
    //         while dz < 7:
    //             print(f"dx: {dx}, dy: {dy}, dz: {dz}")
    //             IK.FindStep(dx, dy, dz, 0, 0)
    //             dz += 1
    //         dy += 1
    //         dz = -7
    //     dx += 1
    //     dy = -7
    //     dz = -7

    // dz limits determined by dx, dy, and curr position->can go high as almost 7
    // dx, dy, dz = 0, 0, -7
    // while dz < 7:
    //             print(f"dx: {dx}, dy: {dy}, dz: {dz}")
    //             IK.FindStep(dx, dy, dz, 0, 0)
    //             dz += 1
};

