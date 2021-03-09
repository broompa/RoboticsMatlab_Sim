clc; clear;
syms t1 t2 t1d t2d
% m1 = 3.473 ; %kg
% m2 = 0.196 ; %kgq_ddot = M\input;
m1 = 0.25; 
m2 = 0.25;
a1 = 3; %m 
a2 = 3; %m
g = 9.81; %m/s^2t2d
f1 = 5.3; %Nm.sec
f2 = 1.1;%Nm.sec

[M,V,G,F,Vm] = term(m1,m2,a1,a2,t1,t2,g,f1,f2,t1d,t2d);
%% control parameters
Kp =[30 , 0 ; 
     0  , 25 ; ];        
Kd = [10, 0 ; 
      0 , 15;];



%% functions

function [M,V,G,F,Vm] = term(m1,m2,a1,a2,theta1,theta2,g,f1,f2,theta1d,theta2d)
    M = [(m1+m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(theta2), m2*a2^2 + m2*a1*a2*cos(theta2);
        m2*a2^2 + m2*a1*a2*cos(theta2) , m2*a2^2; ];
    V = [-m2*a1*a2*(2*theta1d*theta2d + theta2d^2)*sin(theta2);
         m2*a1*a2*(theta1d^2)*sin(theta2)];
    G = [(m1+m2)*a1*g*cos(theta1)+ m2*g*a2*cos(theta1+theta2);
        m2*g*a2*cos(theta1+theta2);];
    F = [f1 , 0 ;
         0 , f2;];
    Vm  = [-m2*a1*a2*sin(theta2)*theta2d , -m2*a1*a2*(theta1d + theta2d)*sin(theta2);
           m2*a1*a2*sin(theta2)*theta1d , 0;];
end