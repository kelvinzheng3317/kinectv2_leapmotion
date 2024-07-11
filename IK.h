#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <math.h>
#include <windows.h>
// from my_teachMover import TeachMover

using namespace std;

#define PI 3.14159265

class IK {
        // TeachMover2 Geometrical Paramaters(inches)
        static constexpr float H = 7.625; // The length of Base to Shoulder
        static constexpr float L = 7.0;   // The length of Shoulder to Elbow and Elbow to Wrist
        static constexpr float LL = 3.8;  // The length of Wrist

        static constexpr float C = 2 * PI;     // Radians of a circle
        static constexpr float B_C = 7072 / C;      // Base motor : 7072 steps in 1 rotation->Base can only go 180 deg ? why is it still be divided by 2 * pi
        static constexpr float S_C = 7072 / C;      // Shoulder motor : 7072 steps in 1 rotation
        static constexpr float E_C = 4158 / C;      // Elbow motor : 4158 steps in 1 rotation
        static constexpr float W_C = 1536 / C;      // Right / Left Wrist motor : 1536 steps in 1 rotation
        static constexpr float G_C = 2330 / C;      // Gripper motor : 2330 steps in 1 rotation

        // Current coordinates
        float x = 7;
        float y = 0;
        float z = 14.626;

    public:
        // initialize robot current coordinates
        IK(float x, float y, float z);

        // Uses the robot's current coordinates to determine what the new stepper values after given change
        vector<int> FindStep(float newX, float newY, float newZ, float directionChange);

        float getCoords();

        // update robots coordinates
        void incrCoords(float x, float y, float z);
};


