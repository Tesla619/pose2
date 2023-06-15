clear all; clc;

Th_1 = 90*pi/180;
Th_2 = 90*pi/180;

L_1 = 0.5;
L_2 = 0.5;

%Simple 2DOF Robot
L (1) = Revolute('d',0,'a', L_1,'alpha',0);
L (2) = Revolute('d',0,'a', L_2,'alpha',0);
Robot = SerialLink(L);
Robot.name = 'RobotArm2DOF';

% Shoulder Joint Simulation
for count = 0 : 1 : 40
    Th_1 = Th_1 + (pi/180);
    Robot.plot([Th_1 Th_2]);
end
 
% Shoulder Joint Simulation
for count = 0 : 1 : 25
    Th_2 = Th_2 + (pi/180);
    Robot.plot([Th_1 Th_2]);
end