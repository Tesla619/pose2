function Automatic_btn_Callback(hObject, eventdata, handles)
% hObject    handle to Automatic_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Flag_Auto = 1

% Initialize DH Parameters

% Twist angle
Alpha1 = degtorad(90);
Alpha2 = degtorad(0);
Alpha3 = degtorad(0);
Alpha4 = degtorad(90);
Alpha5 = degtorad(0);

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

while(eq(Flag_Auto,1))
    
set(handles.Manual_or_Auto,'string',num2str(1));
set(handles.Error_msg,'string','Automatic Mode');
guidata(hObject,handles);
pause(0.5)

% Check if robotic arm is enabled or disabled
Hardware_Interface_sw = str2double(handles.hardware_int.String);

% Execute the below code if robotic arm is enabled
if(eq(Hardware_Interface_sw, 1))
    
    set(handles.Error_msg,'string','Robotic Arm: Enabled');
    
%-------------------------------------------------------------------------%
    
%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(-45);
SetPoint_Theta1 = -45;
SetPoint_Theta1 = interp1([125,-125],[0,210],SetPoint_Theta1);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(10);
SetPoint_Theta2 = 10;
SetPoint_Theta2 = interp1([-10,160],[0,210],SetPoint_Theta2);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-60);
SetPoint_Theta3 = -60;
SetPoint_Theta3 = interp1([-100,110],[0,210],SetPoint_Theta3);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(30);
SetPoint_Theta4 = 30;
SetPoint_Theta4 = interp1([7,173],[0,210],SetPoint_Theta4);

% Gripper
Gripper_Val = 10;
SetPoint_Theta5 = 10;
SetPoint_Theta5 = interp1([0,100],[30,60],SetPoint_Theta5);
SetPoint_Theta5 = interp1([20,80],[0,210],SetPoint_Theta5);

% Wrist Roll
SetPoint_Theta5_FK = degtorad(90);

% Manually compute joints homogenous transformation matricies
TFK_0_1 = [(cos(SetPoint_Theta1_FK)) (-sin(SetPoint_Theta1_FK)*cos(Alpha1)) (sin(SetPoint_Theta1_FK)*sin(Alpha1)) ((a1)*cos(SetPoint_Theta1_FK)); (sin(SetPoint_Theta1_FK)) (cos(SetPoint_Theta1_FK)*cos(Alpha1)) (-cos(SetPoint_Theta1_FK)*sin(Alpha1)) ((a1)*sin(SetPoint_Theta1_FK)); (0) (sin(Alpha1)) (cos(Alpha1)) (d1); (0) (0) (0) (1)];
TFK_1_2 = [(cos(SetPoint_Theta2_FK)) (-sin(SetPoint_Theta2_FK)*cos(Alpha2)) (sin(SetPoint_Theta2_FK)*sin(Alpha2)) ((a2)*cos(SetPoint_Theta2_FK)); (sin(SetPoint_Theta2_FK)) (cos(SetPoint_Theta2_FK)*cos(Alpha2)) (-cos(SetPoint_Theta2_FK)*sin(Alpha2)) ((a2)*sin(SetPoint_Theta2_FK)); (0) (sin(Alpha2)) (cos(Alpha2)) (d2); (0) (0) (0) (1)];
TFK_2_3 = [(cos(SetPoint_Theta3_FK)) (-sin(SetPoint_Theta3_FK)*cos(Alpha3)) (sin(SetPoint_Theta3_FK)*sin(Alpha3)) ((a3)*cos(SetPoint_Theta3_FK)); (sin(SetPoint_Theta3_FK)) (cos(SetPoint_Theta3_FK)*cos(Alpha3)) (-cos(SetPoint_Theta3_FK)*sin(Alpha3)) ((a3)*sin(SetPoint_Theta3_FK)); (0) (sin(Alpha3)) (cos(Alpha3)) (d3); (0) (0) (0) (1)];
TFK_3_4 = [(cos(SetPoint_Theta4_FK)) (-sin(SetPoint_Theta4_FK)*cos(Alpha4)) (sin(SetPoint_Theta4_FK)*sin(Alpha4)) ((a4)*cos(SetPoint_Theta4_FK)); (sin(SetPoint_Theta4_FK)) (cos(SetPoint_Theta4_FK)*cos(Alpha4)) (-cos(SetPoint_Theta4_FK)*sin(Alpha4)) ((a4)*sin(SetPoint_Theta4_FK)); (0) (sin(Alpha4)) (cos(Alpha4)) (d4); (0) (0) (0) (1)];
TFK_4_5 = [(cos(SetPoint_Theta5_FK)) (-sin(SetPoint_Theta5_FK)*cos(Alpha5)) (sin(SetPoint_Theta5_FK)*sin(Alpha5)) ((a5)*cos(SetPoint_Theta5_FK)); (sin(SetPoint_Theta5_FK)) (cos(SetPoint_Theta5_FK)*cos(Alpha5)) (-cos(SetPoint_Theta5_FK)*sin(Alpha5)) ((a5)*sin(SetPoint_Theta5_FK)); (0) (sin(Alpha5)) (cos(Alpha5)) (d5); (0) (0) (0) (1)];

% Manually compute robotic arm homogenous transformation matrix
T_FK = TFK_0_1 * TFK_1_2 * TFK_2_3 * TFK_3_4 * TFK_4_5

% Extract the XYZ position data from the matrix
X_FK = round(T_FK(1,4))
Y_FK = round(T_FK(2,4))
Z_FK = round(T_FK(3,4))

% Update XYZ position textboxes according to the new results
handles.Pos_X.String = num2str(X_FK);
handles.Pos_Y.String = num2str(Y_FK);
handles.Pos_Z.String = num2str(Z_FK);

handles.set_theta1.String = num2str(round(radtodeg(SetPoint_Theta1_FK)));
handles.set_theta2.String = num2str(round(radtodeg(SetPoint_Theta2_FK)));
handles.set_theta3.String = num2str(round(radtodeg(SetPoint_Theta3_FK)));
handles.set_theta4.String = num2str(round(radtodeg(SetPoint_Theta4_FK)));
handles.set_theta5.String = num2str(round(radtodeg(SetPoint_Theta5_FK)));
handles.Gripper_TB.String = num2str(round(Gripper_Val));

set(handles.Slider1,'value',round(radtodeg(SetPoint_Theta1_FK)));
set(handles.Slider2,'value',round(radtodeg(SetPoint_Theta2_FK)));
set(handles.Slider3,'value',round(radtodeg(SetPoint_Theta3_FK)));
set(handles.Slider4,'value',round(radtodeg(SetPoint_Theta4_FK)));
set(handles.Slider5,'value',round(radtodeg(SetPoint_Theta5_FK)));
set(handles.Slider10,'value',round(Gripper_Val));
    
%clc;
%clear;
clear a;
%close all;

% Open arduino object.  EDIT COM PORT ACCORDING TO DEVICE MANAGER
