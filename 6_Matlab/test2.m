close all; clear all; clc;

% Set the 6 angles for the 5DoF robotic arm
theta = [pi/4, pi/4, pi/4, pi/4, pi/4];
% Initialize DH Parameters

% Twist angle
Alpha1 = deg2rad(90);
Alpha2 = deg2rad(0);
Alpha3 = deg2rad(0);
Alpha4 = deg2rad(90);
Alpha5 = deg2rad(0);

% Link length
a1 = 0;
a2 = 165;
a3 = 150;
a4 = 0;
a5 = 0;

% Link offset
d1 = 185;
d2 = -100;
d3 = 100;
d4 = 0;
d5 = 110;

% Build up arm in terms of serial links using the DH params
L(1) = Link([0 d1 a1 Alpha1]);
L(2) = Link([0 d2 a2 Alpha2]);
L(3) = Link([0 d3 a3 Alpha3]);
L(4) = Link([0 d4 a4 Alpha4]);
L(5) = Link([0 d5 a5 Alpha5]);

% Set the joints limits
L(1).qlim=[deg2rad(-120) deg2rad(120)];
L(2).qlim=[deg2rad(0) deg2rad(150)];
L(3).qlim=[deg2rad(-90) deg2rad(90)];
L(4).qlim=[deg2rad(15) deg2rad(165)];
L(5).qlim=[deg2rad(-180) deg2rad(180)];

% Compute all serial links and state the name
Robot = SerialLink(L);
Robot.name = 'Mentor Robotic Arm';

% Plot the robot model
Robot.plot(theta);
