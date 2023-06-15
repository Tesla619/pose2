function varargout = GUI_3(varargin)
% GUI_3 MATLAB code for GUI_3.fig
%      GUI_3, by itself, creates a new GUI_3 or raises the existing
%      singleton*.
%
%      H = GUI_3 returns the handle to a new GUI_3 or the handle to
%      the existing singleton*.
%
%      GUI_3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_3.M with the given input arguments.
%
%      GUI_3('Property','Value',...) creates a new GUI_3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_3_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_3

% Last Modified by GUIDE v2.5 27-Jun-2018 18:39:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_3_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_3_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

%-------------------------------------------------------------------------%

% --- Executes just before GUI_3 is made visible.
function GUI_3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_3 (see VARARGIN)

% Choose default command line output for GUI_3
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = GUI_3_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%-------------------------------------------------------------------------%

% --- Executes on slider movement. Slider for Theta1.
function Slider1_Callback(hObject, eventdata, handles)
% hObject    handle to Slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read value of the slider
disp_theta1 = get(hObject,'value');

% Convert the slider value to a string and update the respective textbox
set(handles.set_theta1,'string',num2str(disp_theta1));
guidata(hObject,handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%-------------------------------------------------------------------------%

% --- Executes on slider movement. Slider for Theta2.
function Slider2_Callback(hObject, eventdata, handles)
% hObject    handle to Slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

disp_theta2 = get(hObject,'value');
set(handles.set_theta2,'string',num2str(disp_theta2));
guidata(hObject,handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%-------------------------------------------------------------------------%

% --- Executes on slider movement. Slider for Theta4.
function Slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

disp_theta4 = get(hObject,'value');
set(handles.set_theta4,'string',num2str(disp_theta4));
guidata(hObject,handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%-------------------------------------------------------------------------%

% --- Executes on slider movement. Slider for Theta3.
function Slider3_Callback(hObject, eventdata, handles)
% hObject    handle to Slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

disp_theta3 = get(hObject,'value');
set(handles.set_theta3,'string',num2str(disp_theta3));
guidata(hObject,handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%-------------------------------------------------------------------------%

% --- Executes on slider movement. Slider for Theta5.
function Slider5_Callback(hObject, eventdata, handles)
% hObject    handle to Slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

disp_theta5 = get(hObject,'value');
set(handles.set_theta5,'string',num2str(disp_theta5));
guidata(hObject,handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%-------------------------------------------------------------------------%

% Executes on button press in btn_Forward. Forward kinematics computations.
function btn_Forward_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read angle values from textboxes and convert to radians
Theta1_FK = degtorad(round(str2double(handles.set_theta1.String)));
Theta2_FK = degtorad(round(str2double(handles.set_theta2.String)));
Theta3_FK = degtorad(round(str2double(handles.set_theta3.String)));
Theta4_FK = degtorad(round(str2double(handles.set_theta4.String)));
Theta5_FK = degtorad(round(str2double(handles.set_theta5.String)));

% Read gripper value from textbox
ThetaG_FK = round(str2double(handles.Gripper_TB.String));

% Check if robotic arm is enabled or disabled
Hardware_Interface_sw = str2double(handles.hardware_int.String);

% Execute the below code if robotic arm is enabled
if(eq(Hardware_Interface_sw, 1))
    
    set(handles.Error_msg,'string','Robotic Arm: Enabled');
    
clc;
%clear;
clear a;
%close all;

% Open arduino object.  EDIT COM PORT ACCORDING TO DEVICE MANAGER
a = arduino('COM5','Mega2560')

% Set all outputs to OFF
writeDigitalPin(a, 'D32', 0);
writeDigitalPin(a, 'D33', 0);
writeDigitalPin(a, 'D34', 0);
writeDigitalPin(a, 'D35', 0);
writeDigitalPin(a, 'D36', 0);
writeDigitalPin(a, 'D37', 0);
writeDigitalPin(a, 'D38', 0);
writeDigitalPin(a, 'D39', 0);
writeDigitalPin(a, 'D40', 0);
writeDigitalPin(a, 'D41', 0);
writeDigitalPin(a, 'D42', 0);
writeDigitalPin(a, 'D43', 0);

writePWMVoltage(a,'D8',0);
writePWMVoltage(a,'D9',0);
writePWMVoltage(a,'D10',0);
writePWMVoltage(a,'D11',0);
writePWMVoltage(a,'D12',0);
writePWMVoltage(a,'D13',0);

% PID Gains
Kp1 = 10;
Kd1 = 0;
Ki1 = 0;

Kp2 = 6;
Kd2 = 0;
Ki2 = 0;

Kp3 = 2;
Kd3 = 0.1;
Ki3 = 0;

Kp4 = 6;
Kd4 = 1;
Ki4 = 0.1;

Kp5 = 0.9;
Kd5 = 0.5;
Ki5 = 0.1;

% Reset, flags, integrals and errors
Integral1 = 0;
Flag1 = 1;
Error1 = 0;

Integral2 = 0;
Flag2 = 1;
Error2 = 0;

Integral3 = 0;
Flag3 = 1;
Error3 = 0;

Integral4 = 0;
Flag4 = 1;
Error4 = 0;

Integral5 = 0;
Flag5 = 1;
Error5 = 0;

% Read angle and gripper values from textboxes

% Body Joint
SetPoint_Theta1 = round(str2double(handles.set_theta1.String));
SetPoint_Theta1 = interp1([122,-122],[0,210],SetPoint_Theta1);

% Shoulder Joint
SetPoint_Theta2 = round(str2double(handles.set_theta2.String));
SetPoint_Theta2 = interp1([-10,160],[0,210],SetPoint_Theta2);

% Elbow Joint
SetPoint_Theta3 = round(str2double(handles.set_theta3.String));
SetPoint_Theta3 = interp1([-100,110],[0,210],SetPoint_Theta3);

% Wrist Pitch Joint
SetPoint_Theta4 = round(str2double(handles.set_theta4.String));
SetPoint_Theta4 = interp1([5,175],[0,210],SetPoint_Theta4);

% Gripper
SetPoint_Theta5 = round(str2double(handles.Gripper_TB.String));
SetPoint_Theta5 = interp1([0,100],[30,60],SetPoint_Theta5);
SetPoint_Theta5 = interp1([20,80],[0,210],SetPoint_Theta5);

% Body Joint Control
while (eq(Flag1,1))

Motor_Angle1 = 0;

% Read potentiometer data and take 10 samples
for i = 1:10

sample1 = readVoltage(a, 'A0');

Motor_Angle1 = Motor_Angle1 + sample1;

end

% Find the average of the samples
AVG_Motor_Angle1 = Motor_Angle1/10;

% Scale analogue input
Angle1_Scaled = interp1([0,3.255],[0,210],AVG_Motor_Angle1);
%Angle1_Scaled = interp1([0,2.5],[0,210],AVG_Motor_Angle1);

% Work out derivative
Errorbefore1 = abs(Error1);
Derivative1 = Error1 - Errorbefore1;

% Find the current error
Error1 = Angle1_Scaled - SetPoint_Theta1;

% Work out the integral
Integral1 = Integral1 + Error1;

if (Error1>0.5)
    
    % Set motor direction
    writeDigitalPin(a, 'D43', 0);
    writeDigitalPin(a, 'D42', 1);
    
    % Adjust PWM output
    PWM1 = 180 + ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    % Clamp PWM output
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    % Clamp PWM output
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    % Scale PWM output and set pin
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
elseif (Error1<-0.5)
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 1);
    
    PWM1 = 180 - ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
% Stop motor if error is within the desired limits and tolerance    
elseif ((lt(Error1,0.5))&&(gt(Error1,-0.5)))
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 0);
    writePWMVoltage(a,'D8',0);
    Flag1 = 0;
end

end

% Shoulder Joint Control
while (eq(Flag2,1))
    
Motor_Angle2 = 0;

for i = 1:10

sample2 = readVoltage(a, 'A1');

Motor_Angle2 = Motor_Angle2 + sample2;

end

AVG_Motor_Angle2 = Motor_Angle2/10;

%Angle2_Scaled = interp1([0.3,2.9],[0,210],AVG_Motor_Angle2);
Angle2_Scaled = interp1([0.3,2.7],[0,210],AVG_Motor_Angle2);

Errorbefore2 = abs(Error2);
Derivative2 = Error2 - Errorbefore2;

Error2 = Angle2_Scaled - SetPoint_Theta2;

Integral2 = Integral2 + Error2;

if (Error2>0.7)
    writeDigitalPin(a, 'D33', 0);
    writeDigitalPin(a, 'D32', 1);
    
    PWM2 = 180 + ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif (Error2<-0.7)
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 1);
    
    PWM2 = 180 - ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif ((lt(Error2,0.7))||(gt(Error2,-0.7)))
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 0);
    writePWMVoltage(a,'D13',0);
    Flag2 = 0;
end

end

% Elbow Joint Control
while (eq(Flag3,1))

Motor_Angle3 = 0;

for i = 1:10

sample3 = readVoltage(a, 'A2');

Motor_Angle3 = Motor_Angle3 + sample3;

end

AVG_Motor_Angle3 = Motor_Angle3/10;

Angle3_Scaled = interp1([0.3,3.4],[0,210],AVG_Motor_Angle3);

Errorbefore3 = abs(Error3);
Derivative3 = Error3 - Errorbefore3;

Error3 = Angle3_Scaled - SetPoint_Theta3;

Integral3 = Integral3 + Error3;

if (Error3>0.9)
    writeDigitalPin(a, 'D39', 0);
    writeDigitalPin(a, 'D38', 1);
    
    PWM3 = 110 + ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif (Error3<-0.9)
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 1);
    
    PWM3 = 110 - ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif ((lt(Error3,0.9))&&(gt(Error3,-0.9)))
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 0);
    writePWMVoltage(a,'D10',0);
    Flag3 = 0;
end

end

% Wrist Pitch Joint Control
while (eq(Flag4,1))

Motor_Angle4 = 0;

for i = 1:10

sample4 = readVoltage(a, 'A4');

Motor_Angle4 = Motor_Angle4 + sample4;

end

AVG_Motor_Angle4 = Motor_Angle4/10;

Angle4_Scaled = interp1([0.9,5],[0,210],AVG_Motor_Angle4);
%Angle4_Scaled = interp1([0,5],[0,210],AVG_Motor_Angle4);

Errorbefore4 = abs(Error4);
Derivative4 = Error4 - Errorbefore4;

Error4 = Angle4_Scaled - SetPoint_Theta4;

Integral4 = Integral4 + Error4;

if (Error4>2.5)
    writeDigitalPin(a, 'D35', 0);
    writeDigitalPin(a, 'D34', 1);
    
    PWM4 = 150 + ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif (Error4<-2.5)
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 1);
    
    PWM4 = 150 - ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif ((lt(Error4,2.5))&&(gt(Error4,-2.5)))
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 0);
    writePWMVoltage(a,'D12',0);
    Flag4 = 0;
end

end

% Gripper Control
while (eq(Flag5,1))

Motor_Angle5 = 0;

for i = 1:10

sample5 = readVoltage(a, 'A5');

Motor_Angle5 = Motor_Angle5 + sample5;

end

AVG_Motor_Angle5 = Motor_Angle5/10;

%Angle5_Scaled = interp1([0.2,1.7],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.4,1.4],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.5,1.2],[0,210],AVG_Motor_Angle5);
Angle5_Scaled = interp1([0.3,1.3],[0,210],AVG_Motor_Angle5);

Errorbefore5 = abs(Error5);
Derivative5 = Error5 - Errorbefore5;

Error5 = Angle5_Scaled - SetPoint_Theta5;

Integral5 = Integral5 + Error5;

if (Error5>3)
    writeDigitalPin(a, 'D41', 0);
    writeDigitalPin(a, 'D40', 1);
    
    PWM5 = 150 + ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif (Error5<-3)
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 1);
    
    PWM5 = 150 - ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif ((lt(Error5,3))&&(gt(Error5,-3)))
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 0);
    writePWMVoltage(a,'D9',0);
    Flag5 = 0;
end

end
    
% Close and clear arduino object
fclose(serial(a.Port));
clear a;
    
% If robotic arm is disbaled, print the following message
elseif(eq(Hardware_Interface_sw, 0))
    
    set(handles.Error_msg,'string','Robotic Arm: Disabled');
  
% If selector switch is not properly adjusted, print the following message
else
    
    set(handles.Error_msg,'string','Hardware interface selector switch not adjusted properly');   
end

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

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

% Incrementing steps of the simulation
inc_FK1 =  Theta1_FK/50;
inc_FK2 =  Theta2_FK/50;
inc_FK3 =  Theta3_FK/50;
inc_FK4 =  Theta4_FK/50;
inc_FK5 =  Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Compute the robotic arm homogenous transformation matrix using function
T_MATLAB = Robot.fkine([Theta1_FK Theta2_FK Theta3_FK Theta4_FK Theta5_FK])

% Manually compute joints homogenous transformation matricies
TFK_0_1 = [(cos(Theta1_FK)) (-sin(Theta1_FK)*cos(Alpha1)) (sin(Theta1_FK)*sin(Alpha1)) ((a1)*cos(Theta1_FK)); (sin(Theta1_FK)) (cos(Theta1_FK)*cos(Alpha1)) (-cos(Theta1_FK)*sin(Alpha1)) ((a1)*sin(Theta1_FK)); (0) (sin(Alpha1)) (cos(Alpha1)) (d1); (0) (0) (0) (1)];
TFK_1_2 = [(cos(Theta2_FK)) (-sin(Theta2_FK)*cos(Alpha2)) (sin(Theta2_FK)*sin(Alpha2)) ((a2)*cos(Theta2_FK)); (sin(Theta2_FK)) (cos(Theta2_FK)*cos(Alpha2)) (-cos(Theta2_FK)*sin(Alpha2)) ((a2)*sin(Theta2_FK)); (0) (sin(Alpha2)) (cos(Alpha2)) (d2); (0) (0) (0) (1)];
TFK_2_3 = [(cos(Theta3_FK)) (-sin(Theta3_FK)*cos(Alpha3)) (sin(Theta3_FK)*sin(Alpha3)) ((a3)*cos(Theta3_FK)); (sin(Theta3_FK)) (cos(Theta3_FK)*cos(Alpha3)) (-cos(Theta3_FK)*sin(Alpha3)) ((a3)*sin(Theta3_FK)); (0) (sin(Alpha3)) (cos(Alpha3)) (d3); (0) (0) (0) (1)];
TFK_3_4 = [(cos(Theta4_FK)) (-sin(Theta4_FK)*cos(Alpha4)) (sin(Theta4_FK)*sin(Alpha4)) ((a4)*cos(Theta4_FK)); (sin(Theta4_FK)) (cos(Theta4_FK)*cos(Alpha4)) (-cos(Theta4_FK)*sin(Alpha4)) ((a4)*sin(Theta4_FK)); (0) (sin(Alpha4)) (cos(Alpha4)) (d4); (0) (0) (0) (1)];
TFK_4_5 = [(cos(Theta5_FK)) (-sin(Theta5_FK)*cos(Alpha5)) (sin(Theta5_FK)*sin(Alpha5)) ((a5)*cos(Theta5_FK)); (sin(Theta5_FK)) (cos(Theta5_FK)*cos(Alpha5)) (-cos(Theta5_FK)*sin(Alpha5)) ((a5)*sin(Theta5_FK)); (0) (sin(Alpha5)) (cos(Alpha5)) (d5); (0) (0) (0) (1)];

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

% This is unnecessary as these are the same angles obtained from the
% sliders/textboxes
handles.set_theta1.String = num2str(round(radtodeg(Theta1_FK)));
handles.set_theta2.String = num2str(round(radtodeg(Theta2_FK)));
handles.set_theta3.String = num2str(round(radtodeg(Theta3_FK)));
handles.set_theta4.String = num2str(round(radtodeg(Theta4_FK)));
handles.set_theta5.String = num2str(round(radtodeg(Theta5_FK)));

set(handles.Slider1,'value',round(radtodeg(Theta1_FK)));
set(handles.Slider2,'value',round(radtodeg(Theta2_FK)));
set(handles.Slider3,'value',round(radtodeg(Theta3_FK)));
set(handles.Slider4,'value',round(radtodeg(Theta4_FK)));
set(handles.Slider5,'value',round(radtodeg(Theta5_FK)));

% Indicate that the simulation is finished
disp('Simulation Ready')

%-------------------------------------------------------------------------%

% --- Executes on button press in btn_Inverse.
function btn_Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read the XYZ position from the textboxes
X_IK = str2double(handles.Pos_X.String);
Y_IK = str2double(handles.Pos_Y.String);
Z_IK = str2double(handles.Pos_Z.String);

% Set plotting variables to zero
Theta_plot_IK1 = degtorad(0);
Theta_plot_IK2 = degtorad(0);
Theta_plot_IK3 = degtorad(0);
Theta_plot_IK4 = degtorad(0);
Theta_plot_IK5 = degtorad(0);

Alpha1 = degtorad(90);
Alpha2 = degtorad(0);
Alpha3 = degtorad(0);
Alpha4 = degtorad(90);
Alpha5 = degtorad(0);

a1 = 0;
a2 = 165;
a3 = 150;
a4 = 0;
a5 = 0;

d1 = 185;
d2 = -100;
d3 = 100;
d4 = 0;
d5 = 110;

L(1) = Link([0 d1 a1 Alpha1]);
L(2) = Link([0 d2 a2 Alpha2]);
L(3) = Link([0 d3 a3 Alpha3]);
L(4) = Link([0 d4 a4 Alpha4]);
L(5) = Link([0 d5 a5 Alpha5]);

L(1).qlim=[deg2rad(-120) deg2rad(120)];
L(2).qlim=[deg2rad(0) deg2rad(150)];
L(3).qlim=[deg2rad(-90) deg2rad(90)];
L(4).qlim=[deg2rad(15) deg2rad(165)];
L(5).qlim=[deg2rad(-180) deg2rad(180)];

Robot = SerialLink(L);
Robot.name = 'Robotic Arm';

% Initialize the transformation matrix. Leave orientation matrix as zero, 
% so system would identify that it is variable.
Trans_mat_IK = [ 0 0 0 round(X_IK); 0 0 0 round(Y_IK); 0 0 0 round(Z_IK); 0 0 0 1]

% Initialize iterative process strating values
q0_1 = degtorad(0);
q0_2 = degtorad(30);
q0_3 = degtorad(0);
q0_4 = degtorad(50);
q0_5 = degtorad(0);

% Call and compute inverse kinematics computations. Mask according to how 
% many joint you have. 
J = Robot.ikine(Trans_mat_IK, 'q0', [q0_1 q0_2 q0_3 q0_4 q0_5], 'mask', [1 1 1 1 1 0])

% Read and store the computed joint values according to the given point
Theta_IK1 = round(radtodeg(J(1)))
Theta_IK2 = round(radtodeg(J(2)))
Theta_IK3 = round(radtodeg(J(3)))
Theta_IK4 = round(radtodeg(J(4)))
Theta_IK5 = round(radtodeg(J(5)))

% Check if arm is disabled or enabled
Hardware_Interface_sw = str2double(handles.hardware_int.String);

% Execute code below if the robotic arm is enabled
if(eq(Hardware_Interface_sw, 1))
    
    set(handles.Error_msg,'string','Robotic Arm: Enabled');

% Clamp any joint angles exceeding the mechanical limits
if(gt(Theta_IK1, 120))
    Theta_IK1 = 120;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end  
    
if(lt(Theta_IK1, -120))
    Theta_IK1 = -120;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end
    
if(gt(Theta_IK2, 150))
    Theta_IK2 = 150;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end
    
if(lt(Theta_IK2, 0))
    Theta_IK2 = 0;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end
    
if(gt(Theta_IK3, 90))
    Theta_IK3 = 90;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end
    
if(lt(Theta_IK3, -90))
    Theta_IK3 = -90;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end
    
if(gt(Theta_IK4, 165))
    Theta_IK4 = 165;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end
    
if(lt(Theta_IK4, 15))
    Theta_IK4 = 15;
    set(handles.Error_msg,'string','The calculated angle exceed mechanical limits - Desired position could not be reached');
end
    
clc;
%clear;
clear a;
%close all;

% Create arduino object. EDIT COM PORT ACCORDING TO DEVICE MANAGER
a = arduino('COM5','Mega2560')

writeDigitalPin(a, 'D32', 0);
writeDigitalPin(a, 'D33', 0);
writeDigitalPin(a, 'D34', 0);
writeDigitalPin(a, 'D35', 0);
writeDigitalPin(a, 'D36', 0);
writeDigitalPin(a, 'D37', 0);
writeDigitalPin(a, 'D38', 0);
writeDigitalPin(a, 'D39', 0);
writeDigitalPin(a, 'D40', 0);
writeDigitalPin(a, 'D41', 0);
writeDigitalPin(a, 'D42', 0);
writeDigitalPin(a, 'D43', 0);

writePWMVoltage(a,'D8',0);
writePWMVoltage(a,'D9',0);
writePWMVoltage(a,'D10',0);
writePWMVoltage(a,'D11',0);
writePWMVoltage(a,'D12',0);
writePWMVoltage(a,'D13',0);

% PID Gains
Kp1 = 10;
Kd1 = 0;
Ki1 = 0;

Kp2 = 6;
Kd2 = 0;
Ki2 = 0;

Kp3 = 2;
Kd3 = 0.1;
Ki3 = 0;

Kp4 = 6;
Kd4 = 1;
Ki4 = 0.1;

Kp5 = 0.9;
Kd5 = 0.5;
Ki5 = 0.1;

% Reset integrals, flags and errors
Integral1 = 0;
Flag1 = 1;
Error1 = 0;

Integral2 = 0;
Flag2 = 1;
Error2 = 0;

Integral3 = 0;
Flag3 = 1;
Error3 = 0;

Integral4 = 0;
Flag4 = 1;
Error4 = 0;

Integral5 = 0;
Flag5 = 1;
Error5 = 0;

% Read, store and map computed angle values
SetPoint_Theta1 = Theta_IK1;
SetPoint_Theta1 = interp1([120,-120],[0,210],SetPoint_Theta1);

SetPoint_Theta2 = Theta_IK2;
SetPoint_Theta2 = interp1([-10,160],[0,210],SetPoint_Theta2);

SetPoint_Theta3 = Theta_IK3;
SetPoint_Theta3 = interp1([-100,110],[0,210],SetPoint_Theta3);

SetPoint_Theta4 = Theta_IK4;
SetPoint_Theta4 = interp1([7,173],[0,210],SetPoint_Theta4);

SetPoint_Theta5 = round(str2double(handles.Gripper_TB.String));
SetPoint_Theta5 = interp1([0,100],[30,60],SetPoint_Theta5);
SetPoint_Theta5 = interp1([20,80],[0,210],SetPoint_Theta5);

% Body Joint Control
while (eq(Flag1,1))

Motor_Angle1 = 0;

for i = 1:10

sample1 = readVoltage(a, 'A0');

Motor_Angle1 = Motor_Angle1 + sample1;

end

AVG_Motor_Angle1 = Motor_Angle1/10;

Angle1_Scaled = interp1([0,3.255],[0,210],AVG_Motor_Angle1);

Errorbefore1 = abs(Error1);
Derivative1 = Error1 - Errorbefore1;

Error1 = Angle1_Scaled - SetPoint_Theta1;

Integral1 = Integral1 + Error1;

if (Error1>0.5)
    writeDigitalPin(a, 'D43', 0);
    writeDigitalPin(a, 'D42', 1);
    
    PWM1 = 180 + ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
elseif (Error1<-0.5)
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 1);
    
    PWM1 = 180 - ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
elseif ((lt(Error1,0.5))&&(gt(Error1,-0.5)))
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 0);
    writePWMVoltage(a,'D8',0);
    Flag1 = 0;
end

end

% Shoulder Joint Control
while (eq(Flag2,1))
    
Motor_Angle2 = 0;

for i = 1:10

sample2 = readVoltage(a, 'A1');

Motor_Angle2 = Motor_Angle2 + sample2;

end

AVG_Motor_Angle2 = Motor_Angle2/10;

Angle2_Scaled = interp1([0.3,2.9],[0,210],AVG_Motor_Angle2);

Errorbefore2 = abs(Error2);
Derivative2 = Error2 - Errorbefore2;

Error2 = Angle2_Scaled - SetPoint_Theta2;

Integral2 = Integral2 + Error2;

if (Error2>0.7)
    writeDigitalPin(a, 'D33', 0);
    writeDigitalPin(a, 'D32', 1);
    
    PWM2 = 180 + ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif (Error2<-0.7)
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 1);
    
    PWM2 = 180 - ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif ((lt(Error2,0.7))||(gt(Error2,-0.7)))
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 0);
    writePWMVoltage(a,'D13',0);
    Flag2 = 0;
end

end

% Elbow Joint Control
while (eq(Flag3,1))

Motor_Angle3 = 0;

for i = 1:10

sample3 = readVoltage(a, 'A2');

Motor_Angle3 = Motor_Angle3 + sample3;

end

AVG_Motor_Angle3 = Motor_Angle3/10;

Angle3_Scaled = interp1([0.3,3.5],[0,210],AVG_Motor_Angle3);

Errorbefore3 = abs(Error3);
Derivative3 = Error3 - Errorbefore3;

Error3 = Angle3_Scaled - SetPoint_Theta3;

Integral3 = Integral3 + Error3;

if (Error3>0.9)
    writeDigitalPin(a, 'D39', 0);
    writeDigitalPin(a, 'D38', 1);
    
    PWM3 = 110 + ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif (Error3<-0.9)
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 1);
    
    PWM3 = 110 - ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif ((lt(Error3,0.9))&&(gt(Error3,-0.9)))
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 0);
    writePWMVoltage(a,'D10',0);
    Flag3 = 0;
end

end

% Wrist Pitch Joint Control
while (eq(Flag4,1))

Motor_Angle4 = 0;

for i = 1:10

sample4 = readVoltage(a, 'A4');

Motor_Angle4 = Motor_Angle4 + sample4;

end

AVG_Motor_Angle4 = Motor_Angle4/10;

Angle4_Scaled = interp1([0.9,5],[0,210],AVG_Motor_Angle4);
%Angle4_Scaled = interp1([0,5],[0,210],AVG_Motor_Angle4);

Errorbefore4 = abs(Error4);
Derivative4 = Error4 - Errorbefore4;

Error4 = Angle4_Scaled - SetPoint_Theta4;

Integral4 = Integral4 + Error4;

if (Error4>2)
    writeDigitalPin(a, 'D35', 0);
    writeDigitalPin(a, 'D34', 1);
    
    PWM4 = 150 + ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif (Error4<-2)
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 1);
    
    PWM4 = 150 - ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif ((lt(Error4,2))&&(gt(Error4,-2)))
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 0);
    writePWMVoltage(a,'D12',0);
    Flag4 = 0;
end

end

% Gripper Control
while (eq(Flag5,1))

Motor_Angle5 = 0;

for i = 1:10

sample5 = readVoltage(a, 'A5');

Motor_Angle5 = Motor_Angle5 + sample5;

end

AVG_Motor_Angle5 = Motor_Angle5/10;

%Angle5_Scaled = interp1([0.2,1.7],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.4,1.4],[0,210],AVG_Motor_Angle5);
Angle5_Scaled = interp1([0.3,1.3],[0,210],AVG_Motor_Angle5);

Errorbefore5 = abs(Error5);
Derivative5 = Error5 - Errorbefore5;

Error5 = Angle5_Scaled - SetPoint_Theta5;

Integral5 = Integral5 + Error5;

if (Error5>3)
    writeDigitalPin(a, 'D41', 0);
    writeDigitalPin(a, 'D40', 1);
    
    PWM5 = 150 + ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif (Error5<-3)
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 1);
    
    PWM5 = 150 - ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif ((lt(Error5,3))&&(gt(Error5,-3)))
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 0);
    writePWMVoltage(a,'D9',0);
    Flag5 = 0;
end

end
    
% Close and clear arduino object
fclose(serial(a.Port));
clear a;

% Set simulation increments
inc_IK1 =  degtorad(Theta_IK1)/50;
inc_IK2 =  degtorad(Theta_IK2)/50;
inc_IK3 =  degtorad(Theta_IK3)/50;
inc_IK4 =  degtorad(Theta_IK4)/50;
inc_IK5 =  degtorad(Theta_IK5)/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK1 = Theta_plot_IK1 + inc_IK1;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK2 = Theta_plot_IK2 + inc_IK2;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK3 = Theta_plot_IK3 + inc_IK3;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK4 = Theta_plot_IK4 + inc_IK4;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK5 = Theta_plot_IK5 + inc_IK5;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Manually compute homogenous transformation matrices of each joint using
% obtained joint angles from the inverse kinematics compuations
TIK_0_1 = [(cos(Theta_plot_IK1)) (-sin(Theta_plot_IK1)*cos(Alpha1)) (sin(Theta_plot_IK1)*sin(Alpha1)) ((a1)*cos(Theta_plot_IK1)); (sin(Theta_plot_IK1)) (cos(Theta_plot_IK1)*cos(Alpha1)) (-cos(Theta_plot_IK1)*sin(Alpha1)) ((a1)*sin(Theta_plot_IK1)); (0) (sin(Alpha1)) (cos(Alpha1)) (d1); (0) (0) (0) (1)];
TIK_1_2 = [(cos(Theta_plot_IK2)) (-sin(Theta_plot_IK2)*cos(Alpha2)) (sin(Theta_plot_IK2)*sin(Alpha2)) ((a2)*cos(Theta_plot_IK2)); (sin(Theta_plot_IK2)) (cos(Theta_plot_IK2)*cos(Alpha2)) (-cos(Theta_plot_IK2)*sin(Alpha2)) ((a2)*sin(Theta_plot_IK2)); (0) (sin(Alpha2)) (cos(Alpha2)) (d2); (0) (0) (0) (1)];
TIK_2_3 = [(cos(Theta_plot_IK3)) (-sin(Theta_plot_IK3)*cos(Alpha3)) (sin(Theta_plot_IK3)*sin(Alpha3)) ((a3)*cos(Theta_plot_IK3)); (sin(Theta_plot_IK3)) (cos(Theta_plot_IK3)*cos(Alpha3)) (-cos(Theta_plot_IK3)*sin(Alpha3)) ((a3)*sin(Theta_plot_IK3)); (0) (sin(Alpha3)) (cos(Alpha3)) (d3); (0) (0) (0) (1)];
TIK_3_4 = [(cos(Theta_plot_IK4)) (-sin(Theta_plot_IK4)*cos(Alpha4)) (sin(Theta_plot_IK4)*sin(Alpha4)) ((a4)*cos(Theta_plot_IK4)); (sin(Theta_plot_IK4)) (cos(Theta_plot_IK4)*cos(Alpha4)) (-cos(Theta_plot_IK4)*sin(Alpha4)) ((a4)*sin(Theta_plot_IK4)); (0) (sin(Alpha4)) (cos(Alpha4)) (d4); (0) (0) (0) (1)];
TIK_4_5 = [(cos(Theta_plot_IK5)) (-sin(Theta_plot_IK5)*cos(Alpha5)) (sin(Theta_plot_IK5)*sin(Alpha5)) ((a5)*cos(Theta_plot_IK5)); (sin(Theta_plot_IK5)) (cos(Theta_plot_IK5)*cos(Alpha5)) (-cos(Theta_plot_IK5)*sin(Alpha5)) ((a5)*sin(Theta_plot_IK5)); (0) (sin(Alpha5)) (cos(Alpha5)) (d5); (0) (0) (0) (1)];

% Manually compute homogenous transformation matrix of the robotic arm
T_IK = TIK_0_1 * TIK_1_2 * TIK_2_3 * TIK_3_4 * TIK_4_5

% Extract XYZ point of the homogenous transformation matrix. This was to
% confirm the the XYZ points remain the same as they were inserted, thus
% confirming a correct compuation
X_FK_new = round(T_IK(1,4))
Y_FK_new = round(T_IK(2,4))
Z_FK_new = round(T_IK(3,4))

% Update XYZ and joint angles textboxes and sliders
handles.Pos_X.String = num2str(X_FK_new);
handles.Pos_Y.String = num2str(Y_FK_new);
handles.Pos_Z.String = num2str(Z_FK_new);

handles.set_theta1.String = num2str(round(Theta_IK1));
handles.set_theta2.String = num2str(round(Theta_IK2));
handles.set_theta3.String = num2str(round(Theta_IK3));
handles.set_theta4.String = num2str(round(Theta_IK4));
handles.set_theta5.String = num2str(round(Theta_IK5));

set(handles.Slider1,'value',round(Theta_IK1));
set(handles.Slider2,'value',round(Theta_IK2));
set(handles.Slider3,'value',round(Theta_IK3));
set(handles.Slider4,'value',round(Theta_IK4));
set(handles.Slider5,'value',round(Theta_IK5));
guidata(hObject,handles);

% Notify that the simulation is ready
disp('Simulation Ready')
    
% Execute upcoming code if robotic arm is disabled
elseif(eq(Hardware_Interface_sw, 0))
    
set(handles.Error_msg,'string','Robotic Arm: Disabled');
    
inc_IK1 =  J(1)/50;
inc_IK2 =  J(2)/50;
inc_IK3 =  J(3)/50;
inc_IK4 =  J(4)/50;
inc_IK5 =  J(5)/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK1 = Theta_plot_IK1 + inc_IK1;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK2 = Theta_plot_IK2 + inc_IK2;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK3 = Theta_plot_IK3 + inc_IK3;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK4 = Theta_plot_IK4 + inc_IK4;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK5 = Theta_plot_IK5 + inc_IK5;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

TIK_0_1 = [(cos(Theta_plot_IK1)) (-sin(Theta_plot_IK1)*cos(Alpha1)) (sin(Theta_plot_IK1)*sin(Alpha1)) ((a1)*cos(Theta_plot_IK1)); (sin(Theta_plot_IK1)) (cos(Theta_plot_IK1)*cos(Alpha1)) (-cos(Theta_plot_IK1)*sin(Alpha1)) ((a1)*sin(Theta_plot_IK1)); (0) (sin(Alpha1)) (cos(Alpha1)) (d1); (0) (0) (0) (1)];
TIK_1_2 = [(cos(Theta_plot_IK2)) (-sin(Theta_plot_IK2)*cos(Alpha2)) (sin(Theta_plot_IK2)*sin(Alpha2)) ((a2)*cos(Theta_plot_IK2)); (sin(Theta_plot_IK2)) (cos(Theta_plot_IK2)*cos(Alpha2)) (-cos(Theta_plot_IK2)*sin(Alpha2)) ((a2)*sin(Theta_plot_IK2)); (0) (sin(Alpha2)) (cos(Alpha2)) (d2); (0) (0) (0) (1)];
TIK_2_3 = [(cos(Theta_plot_IK3)) (-sin(Theta_plot_IK3)*cos(Alpha3)) (sin(Theta_plot_IK3)*sin(Alpha3)) ((a3)*cos(Theta_plot_IK3)); (sin(Theta_plot_IK3)) (cos(Theta_plot_IK3)*cos(Alpha3)) (-cos(Theta_plot_IK3)*sin(Alpha3)) ((a3)*sin(Theta_plot_IK3)); (0) (sin(Alpha3)) (cos(Alpha3)) (d3); (0) (0) (0) (1)];
TIK_3_4 = [(cos(Theta_plot_IK4)) (-sin(Theta_plot_IK4)*cos(Alpha4)) (sin(Theta_plot_IK4)*sin(Alpha4)) ((a4)*cos(Theta_plot_IK4)); (sin(Theta_plot_IK4)) (cos(Theta_plot_IK4)*cos(Alpha4)) (-cos(Theta_plot_IK4)*sin(Alpha4)) ((a4)*sin(Theta_plot_IK4)); (0) (sin(Alpha4)) (cos(Alpha4)) (d4); (0) (0) (0) (1)];
TIK_4_5 = [(cos(Theta_plot_IK5)) (-sin(Theta_plot_IK5)*cos(Alpha5)) (sin(Theta_plot_IK5)*sin(Alpha5)) ((a5)*cos(Theta_plot_IK5)); (sin(Theta_plot_IK5)) (cos(Theta_plot_IK5)*cos(Alpha5)) (-cos(Theta_plot_IK5)*sin(Alpha5)) ((a5)*sin(Theta_plot_IK5)); (0) (sin(Alpha5)) (cos(Alpha5)) (d5); (0) (0) (0) (1)];

T_IK = TIK_0_1 * TIK_1_2 * TIK_2_3 * TIK_3_4 * TIK_4_5

X_FK_new = round(T_IK(1,4))
Y_FK_new = round(T_IK(2,4))
Z_FK_new = round(T_IK(3,4))

handles.Pos_X.String = num2str(X_FK_new);
handles.Pos_Y.String = num2str(Y_FK_new);
handles.Pos_Z.String = num2str(Z_FK_new);

handles.set_theta1.String = num2str(round(radtodeg(J(1))));
handles.set_theta2.String = num2str(round(radtodeg(J(2))));
handles.set_theta3.String = num2str(round(radtodeg(J(3))));
handles.set_theta4.String = num2str(round(radtodeg(J(4))));
handles.set_theta5.String = num2str(round(radtodeg(J(5))));

set(handles.Slider1,'value',round(radtodeg(J(1))));
set(handles.Slider2,'value',round(radtodeg(J(2))));
set(handles.Slider3,'value',round(radtodeg(J(3))));
set(handles.Slider4,'value',round(radtodeg(J(4))));
set(handles.Slider5,'value',round(radtodeg(J(5))));
guidata(hObject,handles);

disp('Simulation Ready')

% Execute upcoming code if selector switch is not adjusted properly
else
    
set(handles.Error_msg,'string','Hardware interface selector switch not adjusted properly');
    
inc_IK1 =  J(1)/50;
inc_IK2 =  J(2)/50;
inc_IK3 =  J(3)/50;
inc_IK4 =  J(4)/50;
inc_IK5 =  J(5)/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK1 = Theta_plot_IK1 + inc_IK1;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK2 = Theta_plot_IK2 + inc_IK2;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK3 = Theta_plot_IK3 + inc_IK3;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK4 = Theta_plot_IK4 + inc_IK4;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_IK5 = Theta_plot_IK5 + inc_IK5;
 
    Robot.plot([Theta_plot_IK1 Theta_plot_IK2 Theta_plot_IK3 Theta_plot_IK4 Theta_plot_IK5]);
    
end

TIK_0_1 = [(cos(Theta_plot_IK1)) (-sin(Theta_plot_IK1)*cos(Alpha1)) (sin(Theta_plot_IK1)*sin(Alpha1)) ((a1)*cos(Theta_plot_IK1)); (sin(Theta_plot_IK1)) (cos(Theta_plot_IK1)*cos(Alpha1)) (-cos(Theta_plot_IK1)*sin(Alpha1)) ((a1)*sin(Theta_plot_IK1)); (0) (sin(Alpha1)) (cos(Alpha1)) (d1); (0) (0) (0) (1)];
TIK_1_2 = [(cos(Theta_plot_IK2)) (-sin(Theta_plot_IK2)*cos(Alpha2)) (sin(Theta_plot_IK2)*sin(Alpha2)) ((a2)*cos(Theta_plot_IK2)); (sin(Theta_plot_IK2)) (cos(Theta_plot_IK2)*cos(Alpha2)) (-cos(Theta_plot_IK2)*sin(Alpha2)) ((a2)*sin(Theta_plot_IK2)); (0) (sin(Alpha2)) (cos(Alpha2)) (d2); (0) (0) (0) (1)];
TIK_2_3 = [(cos(Theta_plot_IK3)) (-sin(Theta_plot_IK3)*cos(Alpha3)) (sin(Theta_plot_IK3)*sin(Alpha3)) ((a3)*cos(Theta_plot_IK3)); (sin(Theta_plot_IK3)) (cos(Theta_plot_IK3)*cos(Alpha3)) (-cos(Theta_plot_IK3)*sin(Alpha3)) ((a3)*sin(Theta_plot_IK3)); (0) (sin(Alpha3)) (cos(Alpha3)) (d3); (0) (0) (0) (1)];
TIK_3_4 = [(cos(Theta_plot_IK4)) (-sin(Theta_plot_IK4)*cos(Alpha4)) (sin(Theta_plot_IK4)*sin(Alpha4)) ((a4)*cos(Theta_plot_IK4)); (sin(Theta_plot_IK4)) (cos(Theta_plot_IK4)*cos(Alpha4)) (-cos(Theta_plot_IK4)*sin(Alpha4)) ((a4)*sin(Theta_plot_IK4)); (0) (sin(Alpha4)) (cos(Alpha4)) (d4); (0) (0) (0) (1)];
TIK_4_5 = [(cos(Theta_plot_IK5)) (-sin(Theta_plot_IK5)*cos(Alpha5)) (sin(Theta_plot_IK5)*sin(Alpha5)) ((a5)*cos(Theta_plot_IK5)); (sin(Theta_plot_IK5)) (cos(Theta_plot_IK5)*cos(Alpha5)) (-cos(Theta_plot_IK5)*sin(Alpha5)) ((a5)*sin(Theta_plot_IK5)); (0) (sin(Alpha5)) (cos(Alpha5)) (d5); (0) (0) (0) (1)];

T_IK = TIK_0_1 * TIK_1_2 * TIK_2_3 * TIK_3_4 * TIK_4_5

X_FK_new = round(T_IK(1,4))
Y_FK_new = round(T_IK(2,4))
Z_FK_new = round(T_IK(3,4))

handles.Pos_X.String = num2str(X_FK_new);
handles.Pos_Y.String = num2str(Y_FK_new);
handles.Pos_Z.String = num2str(Z_FK_new);

handles.set_theta1.String = num2str(round(radtodeg(J(1))));
handles.set_theta2.String = num2str(round(radtodeg(J(2))));
handles.set_theta3.String = num2str(round(radtodeg(J(3))));
handles.set_theta4.String = num2str(round(radtodeg(J(4))));
handles.set_theta5.String = num2str(round(radtodeg(J(5))));

set(handles.Slider1,'value',round(radtodeg(J(1))));
set(handles.Slider2,'value',round(radtodeg(J(2))));
set(handles.Slider3,'value',round(radtodeg(J(3))));
set(handles.Slider4,'value',round(radtodeg(J(4))));
set(handles.Slider5,'value',round(radtodeg(J(5))));
guidata(hObject,handles);

disp('Simulation Ready')

end

%-------------------------------------------------------------------------%

% Represents the textbox for Theta1
function set_theta1_Callback(hObject, eventdata, handles)
% hObject    handle to set_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read value as a string from the textbox
edit_theta1 = get(hObject,'string');

% Covert textbox value to a number and update the respective slider
set(handles.Slider1,'value',str2num(edit_theta1));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of set_theta1 as text
%        str2double(get(hObject,'String')) returns contents of set_theta1 as a double


% --- Executes during object creation, after setting all properties.
function set_theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Represents the textbox for Theta2
function set_theta2_Callback(hObject, eventdata, handles)
% hObject    handle to set_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

edit_theta2 = get(hObject,'string');
set(handles.Slider2,'value',str2num(edit_theta2));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of set_theta2 as text
%        str2double(get(hObject,'String')) returns contents of set_theta2 as a double


% --- Executes during object creation, after setting all properties.
function set_theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Represents the textbox for Theta3
function set_theta3_Callback(hObject, eventdata, handles)
% hObject    handle to set_theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

edit_theta3 = get(hObject,'string');
set(handles.Slider3,'value',str2num(edit_theta3));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of set_theta3 as text
%        str2double(get(hObject,'String')) returns contents of set_theta3 as a double


% --- Executes during object creation, after setting all properties.
function set_theta3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Represents the textbox for Theta4
function set_theta4_Callback(hObject, eventdata, handles)
% hObject    handle to set_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

edit_theta4 = get(hObject,'string');
set(handles.Slider4,'value',str2num(edit_theta4));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of set_theta4 as text
%        str2double(get(hObject,'String')) returns contents of set_theta4 as a double


% --- Executes during object creation, after setting all properties.
function set_theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Respresents the textbox for Theta 5
function set_theta5_Callback(hObject, eventdata, handles)
% hObject    handle to set_theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

edit_theta5 = get(hObject,'string');
set(handles.Slider5,'value',str2num(edit_theta5));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of set_theta5 as text
%        str2double(get(hObject,'String')) returns contents of set_theta5 as a double


% --- Executes during object creation, after setting all properties.
function set_theta5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Respresents x-axis position textbox
function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Represents the y-axis position textbox
function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Represents the z-axis position textbox
function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Represents the error message textbox
function Error_msg_Callback(hObject, eventdata, handles)
% hObject    handle to Error_msg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Error_msg as text
%        str2double(get(hObject,'String')) returns contents of Error_msg as a double


% --- Executes during object creation, after setting all properties.
function Error_msg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Error_msg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% Executes on slider movement. This slider is to disable/enable the arm
function Slider9_Callback(hObject, eventdata, handles)
% hObject    handle to Slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

disp_hardware = get(hObject,'value');

if(disp_hardware == 1)
    hardware_message = 'Enabled';
    
elseif(disp_hardware == 0)
    hardware_message = 'Disabled';
    
else
    hardware_message = 'Wrong Setting';
end

set(handles.hardware_int,'string',num2str(disp_hardware));
guidata(hObject,handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%-------------------------------------------------------------------------%

% Represents the textbox linked with the enable/disable slider
function hardware_int_Callback(hObject, eventdata, handles)
% hObject    handle to hardware_int (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

edit_hardware = get(hObject,'string');
set(handles.Slider9,'value',str2num(edit_hardware));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of hardware_int as text
%        str2double(get(hObject,'String')) returns contents of hardware_int as a double


% --- Executes during object creation, after setting all properties.
function hardware_int_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hardware_int (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% --- Executes on slider movement. Slider for the gripper position
function Slider10_Callback(hObject, eventdata, handles)
% hObject    handle to Slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

disp_gripper = get(hObject,'value');
set(handles.Gripper_TB,'string',num2str(disp_gripper));
guidata(hObject,handles);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%-------------------------------------------------------------------------%

% The textbox for the gripper position
function Gripper_TB_Callback(hObject, eventdata, handles)
% hObject    handle to Gripper_TB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

edit_gripper = get(hObject,'string');
set(handles.Slider10,'value',str2num(edit_gripper));
guidata(hObject,handles);

% Hints: get(hObject,'String') returns contents of Gripper_TB as text
%        str2double(get(hObject,'String')) returns contents of Gripper_TB as a double

% --- Executes during object creation, after setting all properties.
function Gripper_TB_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Gripper_TB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%-------------------------------------------------------------------------%

% --- Executes on button press in Automatic_btn.
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
a = arduino('COM5','Mega2560')

% Set all outputs to OFF
writeDigitalPin(a, 'D32', 0);
writeDigitalPin(a, 'D33', 0);
writeDigitalPin(a, 'D34', 0);
writeDigitalPin(a, 'D35', 0);
writeDigitalPin(a, 'D36', 0);
writeDigitalPin(a, 'D37', 0);
writeDigitalPin(a, 'D38', 0);
writeDigitalPin(a, 'D39', 0);
writeDigitalPin(a, 'D40', 0);
writeDigitalPin(a, 'D41', 0);
writeDigitalPin(a, 'D42', 0);
writeDigitalPin(a, 'D43', 0);

writePWMVoltage(a,'D8',0);
writePWMVoltage(a,'D9',0);
writePWMVoltage(a,'D10',0);
writePWMVoltage(a,'D11',0);
writePWMVoltage(a,'D12',0);
writePWMVoltage(a,'D13',0);

% PID Gains
Kp1 = 10;
Kd1 = 0;
Ki1 = 0;

Kp2 = 6;
Kd2 = 0;
Ki2 = 0;

Kp3 = 2;
Kd3 = 0.1;
Ki3 = 0;

Kp4 = 6;
Kd4 = 1;
Ki4 = 0.1;

Kp5 = 0.9;
Kd5 = 0.5;
Ki5 = 0.1;

% Reset, flags, integrals and errors
Integral1 = 0;
Flag1 = 1;
Error1 = 0;

Integral2 = 0;
Flag2 = 1;
Error2 = 0;

Integral3 = 0;
Flag3 = 1;
Error3 = 0;

Integral4 = 0;
Flag4 = 1;
Error4 = 0;

Integral5 = 0;
Flag5 = 1;
Error5 = 0;

% Body Joint Control
while (eq(Flag1,1))

Motor_Angle1 = 0;

% Read potentiometer data and take 10 samples
for i = 1:10

sample1 = readVoltage(a, 'A0');

Motor_Angle1 = Motor_Angle1 + sample1;

end

% Find the average of the samples
AVG_Motor_Angle1 = Motor_Angle1/10;

% Scale analogue input
Angle1_Scaled = interp1([0,3.255],[0,210],AVG_Motor_Angle1);
%Angle1_Scaled = interp1([0,2.5],[0,210],AVG_Motor_Angle1);

% Work out derivative
Errorbefore1 = abs(Error1);
Derivative1 = Error1 - Errorbefore1;

% Find the current error
Error1 = Angle1_Scaled - SetPoint_Theta1;

% Work out the integral
Integral1 = Integral1 + Error1;

if (Error1>0.5)
    
    % Set motor direction
    writeDigitalPin(a, 'D43', 0);
    writeDigitalPin(a, 'D42', 1);
    
    % Adjust PWM output
    PWM1 = 180 + ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    % Clamp PWM output
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    % Clamp PWM output
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    % Scale PWM output and set pin
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
elseif (Error1<-0.5)
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 1);
    
    PWM1 = 180 - ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
% Stop motor if error is within the desired limits and tolerance    
elseif ((lt(Error1,0.5))&&(gt(Error1,-0.5)))
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 0);
    writePWMVoltage(a,'D8',0);
    Flag1 = 0;
end

end

% Shoulder Joint Control
while (eq(Flag2,1))
    
Motor_Angle2 = 0;

for i = 1:10

sample2 = readVoltage(a, 'A1');

Motor_Angle2 = Motor_Angle2 + sample2;

end

AVG_Motor_Angle2 = Motor_Angle2/10;

%Angle2_Scaled = interp1([0.3,2.9],[0,210],AVG_Motor_Angle2);
Angle2_Scaled = interp1([0.3,2.7],[0,210],AVG_Motor_Angle2);

Errorbefore2 = abs(Error2);
Derivative2 = Error2 - Errorbefore2;

Error2 = Angle2_Scaled - SetPoint_Theta2;

Integral2 = Integral2 + Error2;

if (Error2>0.7)
    writeDigitalPin(a, 'D33', 0);
    writeDigitalPin(a, 'D32', 1);
    
    PWM2 = 180 + ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif (Error2<-0.7)
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 1);
    
    PWM2 = 180 - ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif ((lt(Error2,0.7))||(gt(Error2,-0.7)))
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 0);
    writePWMVoltage(a,'D13',0);
    Flag2 = 0;
end

end

% Elbow Joint Control
while (eq(Flag3,1))

Motor_Angle3 = 0;

for i = 1:10

sample3 = readVoltage(a, 'A2');

Motor_Angle3 = Motor_Angle3 + sample3;

end

AVG_Motor_Angle3 = Motor_Angle3/10;

Angle3_Scaled = interp1([0.3,3.4],[0,210],AVG_Motor_Angle3);

Errorbefore3 = abs(Error3);
Derivative3 = Error3 - Errorbefore3;

Error3 = Angle3_Scaled - SetPoint_Theta3;

Integral3 = Integral3 + Error3;

if (Error3>0.9)
    writeDigitalPin(a, 'D39', 0);
    writeDigitalPin(a, 'D38', 1);
    
    PWM3 = 110 + ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif (Error3<-0.9)
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 1);
    
    PWM3 = 110 - ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif ((lt(Error3,0.9))&&(gt(Error3,-0.9)))
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 0);
    writePWMVoltage(a,'D10',0);
    Flag3 = 0;
end

end

% Wrist Pitch Joint Control
while (eq(Flag4,1))

Motor_Angle4 = 0;

for i = 1:10

sample4 = readVoltage(a, 'A4');

Motor_Angle4 = Motor_Angle4 + sample4;

end

AVG_Motor_Angle4 = Motor_Angle4/10;

Angle4_Scaled = interp1([0.9,5],[0,210],AVG_Motor_Angle4);
%Angle4_Scaled = interp1([0,5],[0,210],AVG_Motor_Angle4);

Errorbefore4 = abs(Error4);
Derivative4 = Error4 - Errorbefore4;

Error4 = Angle4_Scaled - SetPoint_Theta4;

Integral4 = Integral4 + Error4;

if (Error4>2)
    writeDigitalPin(a, 'D35', 0);
    writeDigitalPin(a, 'D34', 1);
    
    PWM4 = 150 + ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif (Error4<-2)
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 1);
    
    PWM4 = 150 - ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif ((lt(Error4,2))&&(gt(Error4,-2)))
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 0);
    writePWMVoltage(a,'D12',0);
    Flag4 = 0;
end

end

% Gripper Control
while (eq(Flag5,1))

Motor_Angle5 = 0;

for i = 1:10

sample5 = readVoltage(a, 'A5');

Motor_Angle5 = Motor_Angle5 + sample5;

end

AVG_Motor_Angle5 = Motor_Angle5/10;

%Angle5_Scaled = interp1([0.2,1.7],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.4,1.4],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.5,1.2],[0,210],AVG_Motor_Angle5);
Angle5_Scaled = interp1([0.3,1.3],[0,210],AVG_Motor_Angle5);

Errorbefore5 = abs(Error5);
Derivative5 = Error5 - Errorbefore5;

Error5 = Angle5_Scaled - SetPoint_Theta5;

Integral5 = Integral5 + Error5;

if (Error5>3)
    writeDigitalPin(a, 'D41', 0);
    writeDigitalPin(a, 'D40', 1);
    
    PWM5 = 150 + ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif (Error5<-3)
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 1);
    
    PWM5 = 150 - ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif ((lt(Error5,3))&&(gt(Error5,-3)))
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 0);
    writePWMVoltage(a,'D9',0);
    Flag5 = 0;
end

end
    
% Close and clear arduino object
fclose(serial(a.Port));
clear a;

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%-------------------------------------------------------------------------%

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(60);
SetPoint_Theta1 = 60;
SetPoint_Theta1 = interp1([125,-125],[0,210],SetPoint_Theta1);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(45);
SetPoint_Theta2 = 45;
SetPoint_Theta2 = interp1([-10,160],[0,210],SetPoint_Theta2);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-20);
SetPoint_Theta3 = -20;
SetPoint_Theta3 = interp1([-100,110],[0,210],SetPoint_Theta3);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(90);
SetPoint_Theta4 = 90;
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
a = arduino('COM5','Mega2560')

% Set all outputs to OFF
writeDigitalPin(a, 'D32', 0);
writeDigitalPin(a, 'D33', 0);
writeDigitalPin(a, 'D34', 0);
writeDigitalPin(a, 'D35', 0);
writeDigitalPin(a, 'D36', 0);
writeDigitalPin(a, 'D37', 0);
writeDigitalPin(a, 'D38', 0);
writeDigitalPin(a, 'D39', 0);
writeDigitalPin(a, 'D40', 0);
writeDigitalPin(a, 'D41', 0);
writeDigitalPin(a, 'D42', 0);
writeDigitalPin(a, 'D43', 0);

writePWMVoltage(a,'D8',0);
writePWMVoltage(a,'D9',0);
writePWMVoltage(a,'D10',0);
writePWMVoltage(a,'D11',0);
writePWMVoltage(a,'D12',0);
writePWMVoltage(a,'D13',0);

% PID Gains
Kp1 = 10;
Kd1 = 0;
Ki1 = 0;

Kp2 = 6;
Kd2 = 0;
Ki2 = 0;

Kp3 = 2;
Kd3 = 0.1;
Ki3 = 0;

Kp4 = 6;
Kd4 = 1;
Ki4 = 0.1;

Kp5 = 0.9;
Kd5 = 0.5;
Ki5 = 0.1;

% Reset, flags, integrals and errors
Integral1 = 0;
Flag1 = 1;
Error1 = 0;

Integral2 = 0;
Flag2 = 1;
Error2 = 0;

Integral3 = 0;
Flag3 = 1;
Error3 = 0;

Integral4 = 0;
Flag4 = 1;
Error4 = 0;

Integral5 = 0;
Flag5 = 1;
Error5 = 0;

% Gripper Control
while (eq(Flag5,1))

Motor_Angle5 = 0;

for i = 1:10

sample5 = readVoltage(a, 'A5');

Motor_Angle5 = Motor_Angle5 + sample5;

end

AVG_Motor_Angle5 = Motor_Angle5/10;

%Angle5_Scaled = interp1([0.2,1.7],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.4,1.4],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.5,1.2],[0,210],AVG_Motor_Angle5);
Angle5_Scaled = interp1([0.3,1.3],[0,210],AVG_Motor_Angle5);

Errorbefore5 = abs(Error5);
Derivative5 = Error5 - Errorbefore5;

Error5 = Angle5_Scaled - SetPoint_Theta5;

Integral5 = Integral5 + Error5;

if (Error5>3)
    writeDigitalPin(a, 'D41', 0);
    writeDigitalPin(a, 'D40', 1);
    
    PWM5 = 150 + ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif (Error5<-3)
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 1);
    
    PWM5 = 150 - ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif ((lt(Error5,3))&&(gt(Error5,-3)))
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 0);
    writePWMVoltage(a,'D9',0);
    Flag5 = 0;
end

end

% Wrist Pitch Joint Control
while (eq(Flag4,1))

Motor_Angle4 = 0;

for i = 1:10

sample4 = readVoltage(a, 'A4');

Motor_Angle4 = Motor_Angle4 + sample4;

end

AVG_Motor_Angle4 = Motor_Angle4/10;

Angle4_Scaled = interp1([0.9,5],[0,210],AVG_Motor_Angle4);
%Angle4_Scaled = interp1([0,5],[0,210],AVG_Motor_Angle4);

Errorbefore4 = abs(Error4);
Derivative4 = Error4 - Errorbefore4;

Error4 = Angle4_Scaled - SetPoint_Theta4;

Integral4 = Integral4 + Error4;

if (Error4>2)
    writeDigitalPin(a, 'D35', 0);
    writeDigitalPin(a, 'D34', 1);
    
    PWM4 = 150 + ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif (Error4<-2)
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 1);
    
    PWM4 = 150 - ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif ((lt(Error4,2))&&(gt(Error4,-2)))
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 0);
    writePWMVoltage(a,'D12',0);
    Flag4 = 0;
end

end

% Elbow Joint Control
while (eq(Flag3,1))

Motor_Angle3 = 0;

for i = 1:10

sample3 = readVoltage(a, 'A2');

Motor_Angle3 = Motor_Angle3 + sample3;

end

AVG_Motor_Angle3 = Motor_Angle3/10;

Angle3_Scaled = interp1([0.3,3.4],[0,210],AVG_Motor_Angle3);

Errorbefore3 = abs(Error3);
Derivative3 = Error3 - Errorbefore3;

Error3 = Angle3_Scaled - SetPoint_Theta3;

Integral3 = Integral3 + Error3;

if (Error3>0.9)
    writeDigitalPin(a, 'D39', 0);
    writeDigitalPin(a, 'D38', 1);
    
    PWM3 = 110 + ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif (Error3<-0.9)
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 1);
    
    PWM3 = 110 - ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif ((lt(Error3,0.9))&&(gt(Error3,-0.9)))
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 0);
    writePWMVoltage(a,'D10',0);
    Flag3 = 0;
end

end

% Shoulder Joint Control
while (eq(Flag2,1))
    
Motor_Angle2 = 0;

for i = 1:10

sample2 = readVoltage(a, 'A1');

Motor_Angle2 = Motor_Angle2 + sample2;

end

AVG_Motor_Angle2 = Motor_Angle2/10;

%Angle2_Scaled = interp1([0.3,2.9],[0,210],AVG_Motor_Angle2);
Angle2_Scaled = interp1([0.3,2.7],[0,210],AVG_Motor_Angle2);

Errorbefore2 = abs(Error2);
Derivative2 = Error2 - Errorbefore2;

Error2 = Angle2_Scaled - SetPoint_Theta2;

Integral2 = Integral2 + Error2;

if (Error2>0.7)
    writeDigitalPin(a, 'D33', 0);
    writeDigitalPin(a, 'D32', 1);
    
    PWM2 = 180 + ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif (Error2<-0.7)
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 1);
    
    PWM2 = 180 - ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif ((lt(Error2,0.7))||(gt(Error2,-0.7)))
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 0);
    writePWMVoltage(a,'D13',0);
    Flag2 = 0;
end

end

% Body Joint Control
while (eq(Flag1,1))

Motor_Angle1 = 0;

% Read potentiometer data and take 10 samples
for i = 1:10

sample1 = readVoltage(a, 'A0');

Motor_Angle1 = Motor_Angle1 + sample1;

end

% Find the average of the samples
AVG_Motor_Angle1 = Motor_Angle1/10;

% Scale analogue input
Angle1_Scaled = interp1([0,3.255],[0,210],AVG_Motor_Angle1);
%Angle1_Scaled = interp1([0,2.5],[0,210],AVG_Motor_Angle1);

% Work out derivative
Errorbefore1 = abs(Error1);
Derivative1 = Error1 - Errorbefore1;

% Find the current error
Error1 = Angle1_Scaled - SetPoint_Theta1;

% Work out the integral
Integral1 = Integral1 + Error1;

if (Error1>0.5)
    
    % Set motor direction
    writeDigitalPin(a, 'D43', 0);
    writeDigitalPin(a, 'D42', 1);
    
    % Adjust PWM output
    PWM1 = 180 + ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    % Clamp PWM output
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    % Clamp PWM output
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    % Scale PWM output and set pin
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
elseif (Error1<-0.5)
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 1);
    
    PWM1 = 180 - ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
% Stop motor if error is within the desired limits and tolerance    
elseif ((lt(Error1,0.5))&&(gt(Error1,-0.5)))
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 0);
    writePWMVoltage(a,'D8',0);
    Flag1 = 0;
end

end
    
% Close and clear arduino object
fclose(serial(a.Port));
clear a;

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%-------------------------------------------------------------------------%

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(45);
SetPoint_Theta1 = 45;
SetPoint_Theta1 = interp1([125,-125],[0,210],SetPoint_Theta1);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(10);
SetPoint_Theta2 = 10;
SetPoint_Theta2 = interp1([-10,160],[0,210],SetPoint_Theta2);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-75);
SetPoint_Theta3 = -75;
SetPoint_Theta3 = interp1([-100,110],[0,210],SetPoint_Theta3);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(120);
SetPoint_Theta4 = 120;
SetPoint_Theta4 = interp1([7,173],[0,210],SetPoint_Theta4);

% Gripper
Gripper_Val = 90;
SetPoint_Theta5 = 90;
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
a = arduino('COM5','Mega2560')

% Set all outputs to OFF
writeDigitalPin(a, 'D32', 0);
writeDigitalPin(a, 'D33', 0);
writeDigitalPin(a, 'D34', 0);
writeDigitalPin(a, 'D35', 0);
writeDigitalPin(a, 'D36', 0);
writeDigitalPin(a, 'D37', 0);
writeDigitalPin(a, 'D38', 0);
writeDigitalPin(a, 'D39', 0);
writeDigitalPin(a, 'D40', 0);
writeDigitalPin(a, 'D41', 0);
writeDigitalPin(a, 'D42', 0);
writeDigitalPin(a, 'D43', 0);

writePWMVoltage(a,'D8',0);
writePWMVoltage(a,'D9',0);
writePWMVoltage(a,'D10',0);
writePWMVoltage(a,'D11',0);
writePWMVoltage(a,'D12',0);
writePWMVoltage(a,'D13',0);

% PID Gains
Kp1 = 10;
Kd1 = 0;
Ki1 = 0;

Kp2 = 6;
Kd2 = 0;
Ki2 = 0;

Kp3 = 2;
Kd3 = 0.1;
Ki3 = 0;

Kp4 = 6;
Kd4 = 1;
Ki4 = 0.1;

Kp5 = 0.9;
Kd5 = 0.5;
Ki5 = 0.1;

% Reset, flags, integrals and errors
Integral1 = 0;
Flag1 = 1;
Error1 = 0;

Integral2 = 0;
Flag2 = 1;
Error2 = 0;

Integral3 = 0;
Flag3 = 1;
Error3 = 0;

Integral4 = 0;
Flag4 = 1;
Error4 = 0;

Integral5 = 0;
Flag5 = 1;
Error5 = 0;

% Body Joint Control
while (eq(Flag1,1))

Motor_Angle1 = 0;

% Read potentiometer data and take 10 samples
for i = 1:10

sample1 = readVoltage(a, 'A0');

Motor_Angle1 = Motor_Angle1 + sample1;

end

% Find the average of the samples
AVG_Motor_Angle1 = Motor_Angle1/10;

% Scale analogue input
Angle1_Scaled = interp1([0,3.255],[0,210],AVG_Motor_Angle1);
%Angle1_Scaled = interp1([0,2.5],[0,210],AVG_Motor_Angle1);

% Work out derivative
Errorbefore1 = abs(Error1);
Derivative1 = Error1 - Errorbefore1;

% Find the current error
Error1 = Angle1_Scaled - SetPoint_Theta1;

% Work out the integral
Integral1 = Integral1 + Error1;

if (Error1>0.5)
    
    % Set motor direction
    writeDigitalPin(a, 'D43', 0);
    writeDigitalPin(a, 'D42', 1);
    
    % Adjust PWM output
    PWM1 = 180 + ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    % Clamp PWM output
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    % Clamp PWM output
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    % Scale PWM output and set pin
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
elseif (Error1<-0.5)
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 1);
    
    PWM1 = 180 - ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
% Stop motor if error is within the desired limits and tolerance    
elseif ((lt(Error1,0.5))&&(gt(Error1,-0.5)))
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 0);
    writePWMVoltage(a,'D8',0);
    Flag1 = 0;
end

end

% Shoulder Joint Control
while (eq(Flag2,1))
    
Motor_Angle2 = 0;

for i = 1:10

sample2 = readVoltage(a, 'A1');

Motor_Angle2 = Motor_Angle2 + sample2;

end

AVG_Motor_Angle2 = Motor_Angle2/10;

%Angle2_Scaled = interp1([0.3,2.9],[0,210],AVG_Motor_Angle2);
Angle2_Scaled = interp1([0.3,2.7],[0,210],AVG_Motor_Angle2);

Errorbefore2 = abs(Error2);
Derivative2 = Error2 - Errorbefore2;

Error2 = Angle2_Scaled - SetPoint_Theta2;

Integral2 = Integral2 + Error2;

if (Error2>0.7)
    writeDigitalPin(a, 'D33', 0);
    writeDigitalPin(a, 'D32', 1);
    
    PWM2 = 180 + ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif (Error2<-0.7)
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 1);
    
    PWM2 = 180 - ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif ((lt(Error2,0.7))||(gt(Error2,-0.7)))
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 0);
    writePWMVoltage(a,'D13',0);
    Flag2 = 0;
end

end

% Elbow Joint Control
while (eq(Flag3,1))

Motor_Angle3 = 0;

for i = 1:10

sample3 = readVoltage(a, 'A2');

Motor_Angle3 = Motor_Angle3 + sample3;

end

AVG_Motor_Angle3 = Motor_Angle3/10;

Angle3_Scaled = interp1([0.3,3.4],[0,210],AVG_Motor_Angle3);

Errorbefore3 = abs(Error3);
Derivative3 = Error3 - Errorbefore3;

Error3 = Angle3_Scaled - SetPoint_Theta3;

Integral3 = Integral3 + Error3;

if (Error3>0.9)
    writeDigitalPin(a, 'D39', 0);
    writeDigitalPin(a, 'D38', 1);
    
    PWM3 = 110 + ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif (Error3<-0.9)
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 1);
    
    PWM3 = 110 - ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif ((lt(Error3,0.9))&&(gt(Error3,-0.9)))
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 0);
    writePWMVoltage(a,'D10',0);
    Flag3 = 0;
end

end

% Wrist Pitch Joint Control
while (eq(Flag4,1))

Motor_Angle4 = 0;

for i = 1:10

sample4 = readVoltage(a, 'A4');

Motor_Angle4 = Motor_Angle4 + sample4;

end

AVG_Motor_Angle4 = Motor_Angle4/10;

Angle4_Scaled = interp1([0.9,5],[0,210],AVG_Motor_Angle4);
%Angle4_Scaled = interp1([0,5],[0,210],AVG_Motor_Angle4);

Errorbefore4 = abs(Error4);
Derivative4 = Error4 - Errorbefore4;

Error4 = Angle4_Scaled - SetPoint_Theta4;

Integral4 = Integral4 + Error4;

if (Error4>2)
    writeDigitalPin(a, 'D35', 0);
    writeDigitalPin(a, 'D34', 1);
    
    PWM4 = 150 + ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif (Error4<-2)
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 1);
    
    PWM4 = 150 - ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif ((lt(Error4,2))&&(gt(Error4,-2)))
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 0);
    writePWMVoltage(a,'D12',0);
    Flag4 = 0;
end

end

% Gripper Control
while (eq(Flag5,1))

Motor_Angle5 = 0;

for i = 1:10

sample5 = readVoltage(a, 'A5');

Motor_Angle5 = Motor_Angle5 + sample5;

end

AVG_Motor_Angle5 = Motor_Angle5/10;

%Angle5_Scaled = interp1([0.2,1.7],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.4,1.4],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.5,1.2],[0,210],AVG_Motor_Angle5);
Angle5_Scaled = interp1([0.3,1.3],[0,210],AVG_Motor_Angle5);

Errorbefore5 = abs(Error5);
Derivative5 = Error5 - Errorbefore5;

Error5 = Angle5_Scaled - SetPoint_Theta5;

Integral5 = Integral5 + Error5;

if (Error5>3)
    writeDigitalPin(a, 'D41', 0);
    writeDigitalPin(a, 'D40', 1);
    
    PWM5 = 150 + ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif (Error5<-3)
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 1);
    
    PWM5 = 150 - ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif ((lt(Error5,3))&&(gt(Error5,-3)))
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 0);
    writePWMVoltage(a,'D9',0);
    Flag5 = 0;
end

end
    
% Close and clear arduino object
fclose(serial(a.Port));
clear a;

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%-------------------------------------------------------------------------%

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(0);
SetPoint_Theta1 = 0;
SetPoint_Theta1 = interp1([125,-125],[0,210],SetPoint_Theta1);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(30);
SetPoint_Theta2 = 30;
SetPoint_Theta2 = interp1([-10,160],[0,210],SetPoint_Theta2);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(30);
SetPoint_Theta3 = 30;
SetPoint_Theta3 = interp1([-100,110],[0,210],SetPoint_Theta3);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(90);
SetPoint_Theta4 = 90;
SetPoint_Theta4 = interp1([7,173],[0,210],SetPoint_Theta4);

% Gripper
Gripper_Val = 90;
SetPoint_Theta5 = 90;
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
a = arduino('COM5','Mega2560')

% Set all outputs to OFF
writeDigitalPin(a, 'D32', 0);
writeDigitalPin(a, 'D33', 0);
writeDigitalPin(a, 'D34', 0);
writeDigitalPin(a, 'D35', 0);
writeDigitalPin(a, 'D36', 0);
writeDigitalPin(a, 'D37', 0);
writeDigitalPin(a, 'D38', 0);
writeDigitalPin(a, 'D39', 0);
writeDigitalPin(a, 'D40', 0);
writeDigitalPin(a, 'D41', 0);
writeDigitalPin(a, 'D42', 0);
writeDigitalPin(a, 'D43', 0);

writePWMVoltage(a,'D8',0);
writePWMVoltage(a,'D9',0);
writePWMVoltage(a,'D10',0);
writePWMVoltage(a,'D11',0);
writePWMVoltage(a,'D12',0);
writePWMVoltage(a,'D13',0);

% PID Gains
Kp1 = 10;
Kd1 = 0;
Ki1 = 0;

Kp2 = 6;
Kd2 = 0;
Ki2 = 0;

Kp3 = 2;
Kd3 = 0.1;
Ki3 = 0;

Kp4 = 6;
Kd4 = 1;
Ki4 = 0.1;

Kp5 = 0.9;
Kd5 = 0.5;
Ki5 = 0.1;

% Reset, flags, integrals and errors
Integral1 = 0;
Flag1 = 1;
Error1 = 0;

Integral2 = 0;
Flag2 = 1;
Error2 = 0;

Integral3 = 0;
Flag3 = 1;
Error3 = 0;

Integral4 = 0;
Flag4 = 1;
Error4 = 0;

Integral5 = 0;
Flag5 = 1;
Error5 = 0;

% Gripper Control
while (eq(Flag5,1))

Motor_Angle5 = 0;

for i = 1:10

sample5 = readVoltage(a, 'A5');

Motor_Angle5 = Motor_Angle5 + sample5;

end

AVG_Motor_Angle5 = Motor_Angle5/10;

%Angle5_Scaled = interp1([0.2,1.7],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.4,1.4],[0,210],AVG_Motor_Angle5);
%Angle5_Scaled = interp1([0.5,1.2],[0,210],AVG_Motor_Angle5);
Angle5_Scaled = interp1([0.3,1.3],[0,210],AVG_Motor_Angle5);

Errorbefore5 = abs(Error5);
Derivative5 = Error5 - Errorbefore5;

Error5 = Angle5_Scaled - SetPoint_Theta5;

Integral5 = Integral5 + Error5;

if (Error5>3)
    writeDigitalPin(a, 'D41', 0);
    writeDigitalPin(a, 'D40', 1);
    
    PWM5 = 150 + ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif (Error5<-3)
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 1);
    
    PWM5 = 150 - ((Kp5*Error5) + (Kd5*Derivative5) + (Ki5*Integral5));
    
    if gt(PWM5,390) 
        PWM5 = 390;
    end
    
    if lt(PWM5,0) 
        PWM5 = 0;
    end
    
    PWM5 = interp1([0,390],[0,5],PWM5);
    writePWMVoltage(a,'D9',PWM5);
    
elseif ((lt(Error5,3))&&(gt(Error5,-3)))
    writeDigitalPin(a, 'D40', 0);
    writeDigitalPin(a, 'D41', 0);
    writePWMVoltage(a,'D9',0);
    Flag5 = 0;
end

end

% Wrist Pitch Joint Control
while (eq(Flag4,1))

Motor_Angle4 = 0;

for i = 1:10

sample4 = readVoltage(a, 'A4');

Motor_Angle4 = Motor_Angle4 + sample4;

end

AVG_Motor_Angle4 = Motor_Angle4/10;

Angle4_Scaled = interp1([0.9,5],[0,210],AVG_Motor_Angle4);
%Angle4_Scaled = interp1([0,5],[0,210],AVG_Motor_Angle4);

Errorbefore4 = abs(Error4);
Derivative4 = Error4 - Errorbefore4;

Error4 = Angle4_Scaled - SetPoint_Theta4;

Integral4 = Integral4 + Error4;

if (Error4>2)
    writeDigitalPin(a, 'D35', 0);
    writeDigitalPin(a, 'D34', 1);
    
    PWM4 = 150 + ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif (Error4<-2)
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 1);
    
    PWM4 = 150 - ((Kp4*Error4) + (Kd4*Derivative4) + (Ki4*Integral4));
    
    if gt(PWM4,390) 
        PWM4 = 390;
    end
    
    if lt(PWM4,0) 
        PWM4 = 0;
    end
    
    PWM4 = interp1([0,390],[0,5],PWM4);
    writePWMVoltage(a,'D12',PWM4);
    
elseif ((lt(Error4,2))&&(gt(Error4,-2)))
    writeDigitalPin(a, 'D34', 0);
    writeDigitalPin(a, 'D35', 0);
    writePWMVoltage(a,'D12',0);
    Flag4 = 0;
end

end

% Elbow Joint Control
while (eq(Flag3,1))

Motor_Angle3 = 0;

for i = 1:10

sample3 = readVoltage(a, 'A2');

Motor_Angle3 = Motor_Angle3 + sample3;

end

AVG_Motor_Angle3 = Motor_Angle3/10;

Angle3_Scaled = interp1([0.3,3.4],[0,210],AVG_Motor_Angle3);

Errorbefore3 = abs(Error3);
Derivative3 = Error3 - Errorbefore3;

Error3 = Angle3_Scaled - SetPoint_Theta3;

Integral3 = Integral3 + Error3;

if (Error3>0.9)
    writeDigitalPin(a, 'D39', 0);
    writeDigitalPin(a, 'D38', 1);
    
    PWM3 = 110 + ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif (Error3<-0.9)
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 1);
    
    PWM3 = 110 - ((Kp3*Error3) + (Kd3*Derivative3) + (Ki3*Integral3));
    
    if gt(PWM3,390) 
        PWM3 = 390;
    end
    
    if lt(PWM3,0) 
        PWM3 = 0;
    end
    
    PWM3 = interp1([0,390],[0,5],PWM3);
    writePWMVoltage(a,'D10',PWM3);
    
elseif ((lt(Error3,0.9))&&(gt(Error3,-0.9)))
    writeDigitalPin(a, 'D38', 0);
    writeDigitalPin(a, 'D39', 0);
    writePWMVoltage(a,'D10',0);
    Flag3 = 0;
end

end

% Shoulder Joint Control
while (eq(Flag2,1))
    
Motor_Angle2 = 0;

for i = 1:10

sample2 = readVoltage(a, 'A1');

Motor_Angle2 = Motor_Angle2 + sample2;

end

AVG_Motor_Angle2 = Motor_Angle2/10;

%Angle2_Scaled = interp1([0.3,2.9],[0,210],AVG_Motor_Angle2);
Angle2_Scaled = interp1([0.3,2.7],[0,210],AVG_Motor_Angle2);

Errorbefore2 = abs(Error2);
Derivative2 = Error2 - Errorbefore2;

Error2 = Angle2_Scaled - SetPoint_Theta2;

Integral2 = Integral2 + Error2;

if (Error2>0.7)
    writeDigitalPin(a, 'D33', 0);
    writeDigitalPin(a, 'D32', 1);
    
    PWM2 = 180 + ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif (Error2<-0.7)
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 1);
    
    PWM2 = 180 - ((Kp2*Error2) + (Kd2*Derivative2) + (Ki2*Integral2));
    
    if gt(PWM2,390) 
        PWM2 = 390;
    end
    
    if lt(PWM2,0) 
        PWM2 = 0;
    end
    
    PWM2 = interp1([0,390],[0,5],PWM2);
    writePWMVoltage(a,'D13',PWM2);
    
elseif ((lt(Error2,0.7))||(gt(Error2,-0.7)))
    writeDigitalPin(a, 'D32', 0);
    writeDigitalPin(a, 'D33', 0);
    writePWMVoltage(a,'D13',0);
    Flag2 = 0;
end

end

% Body Joint Control
while (eq(Flag1,1))

Motor_Angle1 = 0;

% Read potentiometer data and take 10 samples
for i = 1:10

sample1 = readVoltage(a, 'A0');

Motor_Angle1 = Motor_Angle1 + sample1;

end

% Find the average of the samples
AVG_Motor_Angle1 = Motor_Angle1/10;

% Scale analogue input
Angle1_Scaled = interp1([0,3.255],[0,210],AVG_Motor_Angle1);
%Angle1_Scaled = interp1([0,2.5],[0,210],AVG_Motor_Angle1);

% Work out derivative
Errorbefore1 = abs(Error1);
Derivative1 = Error1 - Errorbefore1;

% Find the current error
Error1 = Angle1_Scaled - SetPoint_Theta1;

% Work out the integral
Integral1 = Integral1 + Error1;

if (Error1>0.5)
    
    % Set motor direction
    writeDigitalPin(a, 'D43', 0);
    writeDigitalPin(a, 'D42', 1);
    
    % Adjust PWM output
    PWM1 = 180 + ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    % Clamp PWM output
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    % Clamp PWM output
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    % Scale PWM output and set pin
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
elseif (Error1<-0.5)
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 1);
    
    PWM1 = 180 - ((Kp1*Error1) + (Kd1*Derivative1) + (Ki1*Integral1));
    
    if gt(PWM1,390) 
        PWM1 = 390;
    end
    
    if lt(PWM1,0) 
        PWM1 = 0;
    end
    
    PWM1 = interp1([0,390],[0,5],PWM1);
    writePWMVoltage(a,'D8',PWM1);
    
% Stop motor if error is within the desired limits and tolerance    
elseif ((lt(Error1,0.5))&&(gt(Error1,-0.5)))
    writeDigitalPin(a, 'D42', 0);
    writeDigitalPin(a, 'D43', 0);
    writePWMVoltage(a,'D8',0);
    Flag1 = 0;
end

end
    
% Close and clear arduino object
fclose(serial(a.Port));
clear a;

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%-------------------------------------------------------------------------%
    
% If robotic arm is disbaled, print the following message
elseif(eq(Hardware_Interface_sw, 0))
    
    set(handles.Error_msg,'string','Robotic Arm: Disabled');
    
%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(-45);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(10);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-60);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(30);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Wrist Roll Joint Simulation
% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(60);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(45);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-20);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(90);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(45);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(10);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-75);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(120);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(0);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(30);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(30);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(90);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')
  
% If selector switch is not properly adjusted, print the following message
else
    
    set(handles.Error_msg,'string','Hardware interface selector switch not adjusted properly');

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(-45);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(10);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-60);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(30);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Wrist Roll Joint Simulation
% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(60);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(45);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-20);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(90);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%%%---------- Change the values below for different response ----------%%%
%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(45);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(10);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(-75);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(120);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

%--- ALWAYS assign the same value for each joint for proper operation ---%
% Body Joint
SetPoint_Theta1_FK = degtorad(0);

% Shoulder Joint
SetPoint_Theta2_FK = degtorad(30);

% Elbow Joint
SetPoint_Theta3_FK = degtorad(30);

% Wrist Pitch Joint
SetPoint_Theta4_FK = degtorad(90);

% Gripper
Gripper_Val = 0;

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

% Set variables to 0 radians
Theta_plot_FK1 = degtorad(0);
Theta_plot_FK2 = degtorad(0);
Theta_plot_FK3 = degtorad(0);
Theta_plot_FK4 = degtorad(0);
Theta_plot_FK5 = degtorad(0);

% Incrementing steps of the simulation
inc_FK1 =  SetPoint_Theta1_FK/50;
inc_FK2 =  SetPoint_Theta2_FK/50;
inc_FK3 =  SetPoint_Theta3_FK/50;
inc_FK4 =  SetPoint_Theta4_FK/50;
inc_FK5 =  SetPoint_Theta5_FK/50;

% Body Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK1 = Theta_plot_FK1 + inc_FK1;
 
    % Plot the robotic arm with the current angle
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Shoulder Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK2 = Theta_plot_FK2 + inc_FK2;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Elbow Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK3 = Theta_plot_FK3 + inc_FK3;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Pitch Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK4 = Theta_plot_FK4 + inc_FK4;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Wrist Roll Joint Simulation
for count = 0 : 1 : 49
    
    Theta_plot_FK5 = Theta_plot_FK5 + inc_FK5;
 
    Robot.plot([Theta_plot_FK1 Theta_plot_FK2 Theta_plot_FK3 Theta_plot_FK4 Theta_plot_FK5]);
    
end

% Indicate that the simulation is finished
disp('Simulation Ready')

end

%-------------------------------------------------------------------------%

Check_Mode = str2num(handles.Manual_or_Auto.String)

if(eq(Check_Mode, 0))
    
   Flag_Auto = 0
   set(handles.Error_msg,'string','Auto Mode disabled');
    
end

end

%-------------------------------------------------------------------------%

% --- Executes on button press in Manual_btn.
function Manual_btn_Callback(hObject, eventdata, handles)
% hObject    handle to Manual_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Flag_Auto = 0;

set(handles.Manual_or_Auto,'string',num2str(0));
set(handles.Error_msg,'string','Manual Mode');
guidata(hObject,handles);

%-------------------------------------------------------------------------%

% Represents the manual or auto state textbox
function Manual_or_Auto_Callback(hObject, eventdata, handles)
% hObject    handle to Manual_or_Auto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Manual_or_Auto as text
%        str2double(get(hObject,'String')) returns contents of Manual_or_Auto as a double


% --- Executes during object creation, after setting all properties.
function Manual_or_Auto_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Manual_or_Auto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
