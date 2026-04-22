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

/*
Joint Positions for Squares:
Gripper Right Open: 6.50
Gripper Right Closed: -6.50

Gripper Left Open:
Gripper Left Closed:

Right Above Column H: 
    shoulderR.queueMove(122.25);
    bicepR.queueMove(-7.00);
    elbowR.queueMove(-46.46);
    wristR.queueMove(4.55);
    handR.queueMove(23.72);

Right Pick Up Column H:
    shoulderR.queueMove(87.46);
    bicepR.queueMove(-7.17);
    elbowR.queueMove(-40.91);
    wristR.queueMove(7.48);
    handR.queueMove(47.45);

Right Above Column G:
    shoulderR.queueMove(89.16);
    bicepR.queueMove(-15.61);
    elbowR.queueMove(6.19);
    wristR.queueMove(6.82);
    handR.queueMove(2.27);
    
Right Pick Up Column G:
    shoulderR.queueMove(77.07);
    bicepR.queueMove(-15.61);
    elbowR.queueMove(-11.41);
    wristR.queueMove(7.15);
    handR.queueMove(26.32);

Right Above Column F:
    shoulderR.queueMove(80.16);
    bicepR.queueMove(-29.72);
    elbowR.queueMove(1.30);
    wristR.queueMove(0.00);
    handR.queueMove(29.57);

Right Pick Up Column F:
    shoulderR.queueMove(68.16);
    bicepR.queueMove(-29.72);
    elbowR.queueMove(1.30);
    wristR.queueMove(0.00);
    handR.queueMove(29.57);

Right Above Column E:
    shoulderR.queueMove(91.44);
    bicepR.queueMove(-44.58);
    elbowR.queueMove(-0.81);
    wristR.queueMove(0.97);
    handR.queueMove(6.50);

Right Pick Up Column E:
    shoulderR.queueMove(77.44);
    bicepR.queueMove(-44.58);
    elbowR.queueMove(-0.81);
    wristR.queueMove(0.97);
    handR.queueMove(13.50);

Right Above Column D:
    shoulderR.queueMove(84.76);
    bicepR.queueMove(-58.47);
    elbowR.queueMove(5.71);
    wristR.queueMove(0.00);
    handR.queueMove(0.65);

Right Pick Up Column D:
    shoulderR.queueMove(56.56);
    bicepR.queueMove(-58.47);
    elbowR.queueMove(5.87);
    wristR.queueMove(-5.20);
    handR.queueMove(32.18);


*/

#endif