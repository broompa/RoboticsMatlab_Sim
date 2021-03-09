clc  ;
clear ;
% clf;
%% Parameters
m1 = 20 ; m2 = 15 ; m3 = 15; m4 = 0.5; %kg
l0 = 0; l1 = 0.2 ; l2 = 0.25; %m
r1 = 0.1 ; r2 = 0.125; %m;
Iz1 = 0.27 ; Iz2 = 0.31 ; Iz3 = 0.02 ; Iz4 = 10^-4; %kgm^2

alp = Iz1 + (r1^2)*m1 + (l1^2)*m2 + (l1^2)*m3 + (l1^2)*m4;
beta = Iz1 + Iz2 + Iz3 + Iz4 + (l2^2)*m3+ (l2^2)*m4 +m2*(r2^2);
gam = l1*l2*m3 + l1*l2*m4 + l1*m2*r2;
del = Iz3 + Iz4; 

g = 9.81;
k1 = 600 ; k2 = 100;
q0 = [0;0;0;0];
d1 = 1.5; %m
theta4 = 0;
d3 = l0;

syms theta1 theta2 theta3 d4 
%% Computations
% func(alpha, a, d, theta)
T = func(0,l1,d1,theta1)*func(pi,l2,0,theta2)*func(0,0,d3,theta3)*func(0,0,d4,0);

Tb = readtable("Waypoints.xlsx");

states= [];
timeS = [];
for i= 1:size(Tb)*[1;0] 
    x = Tb(i,2).Variables; y = Tb(i,3).Variables; z = Tb(i,4).Variables;
    [q1,q2,q4] = Inv_Kin(x,y,z,l1,l2,d1,d3);
    states = [states; [q1,q2,0,q4]];
    timeS = [timeS; Tb(i,1).Variables];
end
clear x y z q1 q2 q3;

% generate cts states
Ct_States = [0,0,0,0;];
Ct_timeS = [0];
Ct_States1 = [];
for i = 2:size(states)*[1;0]
    x = mtraj(@tpoly,states(i-1,:),states(i,:),50);
    t = linspace(timeS(i-1,:),timeS(i,:),50)';
    Ct_States = [Ct_States;x];
    Ct_timeS = [Ct_timeS;t]; 
end

method = "spline";

x1 = interpn(timeS',states(:,1)',Ct_timeS,method);
x2 = interpn(timeS',states(:,2)',Ct_timeS,method);
x3 = interpn(timeS',states(:,3)',Ct_timeS,method);
x4 = interpn(timeS',states(:,4)',Ct_timeS,method);
Ct_States1 = [Ct_States1;[x1, x2, x3, x4]];

clear x1 x2 x3 x4 x t;
disp("Now run the model.slx then, show_trajectory.m")

% ================ Comparision of different Interpolation Methods ===============
% plot(timeS,states(:,2),'o');
% hold on;
% plot(Ct_timeS,Ct_States(:,2));
% plot(Ct_timeS,Ct_States1(:,2));
% legend("markers","mtraj","interp");    

%% functions
function [q1, q2, q4]  = Inv_Kin(x,y,z,l1,l2,d1,d3)  %Inverse Kinematics Mapping for given instance
    q2 = acos((x^2 + y^2 - l1^2 -l2^2)/(2*l1*l2));
    q1 = atan2(y,x) - atan2(l2*sin(q2),l1+l2*cos(q2));
    q4 = d1 - z - d3; %may have to change after arrival of mail.
end

function T = func(alpha,a,d,theta)
    td = [[ 1, 0, 0, 0]
    [ 0, 1, 0, 0]
    [ 0, 0, 1, d]
    [ 0, 0, 0, 1]
     ];
    ta = [[ 1, 0, 0, a]
    [ 0, 1, 0, 0]
    [ 0, 0, 1, 0]
    [ 0, 0, 0, 1]
     ];
    T= trotz(theta)*td*ta*trotx(alpha);
end