clf ;
%% Plotting 
Interp_traj= [];
disp(0)
for i = 1:length(Ct_States1)
    [x,y,z] = For_Kin(Ct_States1(i,1),Ct_States1(i,2),Ct_States1(i,4),l1,l2,d1);
    Interp_traj = [Interp_traj; [x,y,z]];
end
% out.M_Position : [x,y,z]
disp(1)
plot3(Interp_traj(:,1),Interp_traj(:,2),Interp_traj(:,3));
hold on;
plot3(out.M_Position(:,1),out.M_Position(:,2),out.M_Position(:,3));
plot3(Tb(:,2).Variables,Tb(:,3).Variables,Tb(:,4).Variables,"rx");
legend("Interpolated trajectory","Actual Trajectory","Waypoints");
disp("Plotted");

%% functions

function [x,y,z]  = For_Kin(q1,q2,q4,l1,l2,d1)
    x = l1*cos(q1) + l2*cos(q1+q2);
    y = l1*sin(q1) + l2*sin(q1+q2);
    z = d1 - q4;
end