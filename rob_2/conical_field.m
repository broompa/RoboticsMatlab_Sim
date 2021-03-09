parameters;

% Paremeters
rEffec = 5;
n = 10000;
ka = 200;

step = 0.5;
[X,Y] = meshgrid(0:step:30);
U = conical(X,Y,[20,20],o,rEffec,n,ka);
[dx,dy] = gradient(-U,step);

quiver(X,Y,dx,dy);
hold on;
contour(X,Y,-U);

% actual path computation 
q = [0,0];
qg = [20,20];
threshold = 0.1;   
t_step = 0.001;
qs = [];
iteration = 1;
while norm(qg - q) > threshold
    pause(0.0001);
    plot(q(1),q(2),'*r');
    q_dot = ka.*(qg - q)./norm(qg - q) ;
    [q_o,p] = closest_Point(q,o);
        if (p <= rEffec )
            rep = -(n/(p^3))*(1/p - 1/rEffec).*[(q_o(1) - q(1)), (q_o(2) - q(2));];
        else
            rep = [0;0] ;
        end
    q_dot = q_dot+ rep;
    q = q + q_dot*t_step;  
%     F(iteration) = getframe;
    iteration = iteration + 1;
end

disp("Iterations: ");
disp(iteration);


f2 = figure;
surf(X,Y,U);

 
% % F - frames
% writerObj = VideoWriter('Field_2.avi');
% % writerObj.FrameRate = max(1/delay - 10, 20);
% writerObj.FrameRate = 20;
% open(writerObj);
% for i = 1:length(F)
%     frame = F(i);
%     writeVideo(writerObj,frame);
% end
% close(writerObj);
% %



function U = conical(X,Y,goal,o,rEffec,n,ka)
    Attractive = attractive(X,Y,goal,ka);
    Repulsive = zeros(size(X));
    for i = 1:length(X(:,1))
        for j = 1:length(X(1,:))
            Repulsive(i,j) = repulsive([X(i,j),Y(i,j)],o,rEffec,n);
        end
    end
%     Attractive(Repulsive ~= n) = n;
    U =  Attractive+Repulsive;
end

function a = attractive(X,Y,goal,ka)
    a = ka*sqrt((goal(1) - X).^2 + (goal(2) - Y).^2);
end

function r = repulsive(p,o,rEff,n)
   dist = inf ; index = -1;
   for i = 1:length(o(:,1))
       d = sqrt((p(1) - o(i,1))^2 + (p(2) - o(i,2))^2);
       if (dist > d )
           dist = d ;
           index = i;
       end
   end
   dist = dist - o(index,3);
   if (dist <= 0  )
        r = n;
   elseif (dist > rEff)
        r = 0;
   else 
        r = min(n,(n/2)*((1/dist) - (1/rEff))^2);
   end
end

function [p,dist] = closest_Point(q,o)
    dist = inf ; index = -1;
    for i = 1:length(o(:,1))
        d = sqrt((q(1) - o(i,1))^2 + (q(2) - o(i,2))^2) ;
        if dist > d 
            dist = d;
            index = i;
        end        
    end
    dist = dist -o(index,3);
    theta = atan2(q(2) - o(index,2),q(1)-o(index,1));
    p = [o(index,1) + o(index,3)*cos(theta);
         o(index,2) + o(index,3)*sin(theta);];
end
