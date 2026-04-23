#ifndef MotorIDs_h
#define MotorIDs_h

enum MotorID {

    // Serial Bus 1
    shoulderspin_right = 7,
    biceplift_right = 1,
    elbow_right = 2,
    wristspin_right = 28,
    handcurl_right = 3,
    gripper_right = 26,

    // Serial Bus 2
    shoulderspin_left = 15,
    biceplift_left = 32,
    elbow_left = 8,
    wristspin_left = 25,
    handcurl_left = 14,
    gripper_left = 35,
};

float ROpen = 7.50;  // Right hand open angle
float RClosed = -9.50; // Right hand closed angle
float LOpen = -5.50;  // Left hand open angle
float LClosed = 19.50; // Left hand closed angle

struct ArmPosition {
    float angles[5];  // shoulder, bicep, elbow, wrist, hand
};

struct ColumnPositions {
    ArmPosition above;
    ArmPosition pick;
};

// Right arm column positions (H, G, F, E, D)
const ColumnPositions RIGHT_COLUMNS[] = {
    { {94.38, -7.61, -6.03, -2.60, 12.02},   {63.90, -7.61, -11.08, -1.30, 41.93} },  // H
    { {72.54, -20.89, 18.42, -7.80, 18.52},  {47.76, -20.89, 18.42, -8.45, 37.38} }, // G
    { {77.91, -34.75, 8.64, -3.25, 15.92},   {53.79, -34.75, 8.48, -13.65, 39.00} }, // F
    { {77.43, -45.61, 8.00, -9.42, 15.60},    {65.36, 113.34, -5.00, -12.02, 45.32} },  // E
    { {84.76, -58.47, 5.71, 0.00, 0.65},     {56.56, -58.47, 5.87, -5.20, 32.18} },  // D
};

// Left arm column positions - (A, B, C, D, E)
const ColumnPositions LEFT_COLUMNS[] = {
    { {-94.38, 9.00, 6.03, 2.60, -12.02}, {-68.90, 9.00, 11.08, 1.30, -41.93} },  // A
    { {-81.83, 14.81, -1.30, 2.60, -10.07}, {-58.84, 14.81, -4.08, 5.85, -28.92} }, // B
    { {-80.16, 15.61, -1.30, 0.00, -29.57},    {-68.16, 15.61, -1.30, 0.00, -29.57} },    // C
    { {-91.44, 29.72, 0.81, -0.97, -6.50},   {-77.44, 29.72, 0.81, -0.97, -13.50} },  // D
    { {-84.76, 44.58, -5.71, 0.00, -0.65},     {-56.56, 44.58, -5.87, 5.20, -32.18} },    // E
};

const char RIGHT_COLUMN_KEYS[] = {'H', 'G', 'F', 'E', 'D'};
const char LEFT_COLUMN_KEYS[] = {'A', 'B', 'C', 'D', 'E'};
const int NUM_COLUMNS = 5;

#endif