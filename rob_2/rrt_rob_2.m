parameters
plot(s(:,1),s(:,2),'*b');
grid on;
hold on;
plot(g(:,1),g(:,2),'or');
theta = 0:2*pi/100:2*pi;
for i = 1:length(o(:,1)) % obstacle boundary
    x_s = o(i,1) + o(i,3)*cos(theta);
    y_s = o(i,2) + o(i,3)*sin(theta);
    plot(x_s,y_s);
end

xlim([0,30]);
ylim([0,30]);

% rrt: ----------------------
s_Parents = [1, 1;]; %[node, parent] % tree tracking
g_Parents = [1 ,1;]; %[child, parent] 

m2 = 1; m1 = 1; % condition for looping, should add another node.
step = 1;  % delta 
iteration = 1;
delay = 0.02;
while (m1 == 1 && m2 == 1)
    disp(iteration);
    pause(delay);
    [m1,s,s_Parents] = explore(s,g,o,step,s_Parents,g_Parents); % explore source tree 
    plot(s(:,1),s(:,2),'*b'); % draw blue stars
    if m1 == 1 % if not connected
        p1_index = s_Parents(end,1);p1_parent_index = s_Parents(end,2); % indexes
        plot([s(p1_index,1),s(p1_parent_index,1)],[s(p1_index,2),s(p1_parent_index,2)],'b'); % draw an edge for new node source tree
        [m2,g,g_Parents] = explore(g,s,o,step,g_Parents,s_Parents); % expand goal tree
        if m2 == 1 % if not connected 
            p2_index = g_Parents(end,1);p2_parent_index = g_Parents(end,2); % node indexes
            plot([g(p2_index,1),g(p2_parent_index,1)],[g(p2_index,2),g(p2_parent_index,2)],'r'); %draw an edge for new node.
        end
    end
    plot(g(:,1),g(:,2),'or');  % draw red circles
%     F(iteration) = getframe;
    iteration = iteration +1;
end
% Some extra frame for pause
for i = 1:10
    F(iteration) = getframe;
    iteration = iteration + 1;
end

% F - frames
writerObj = VideoWriter('rrt_animation.avi');
% writerObj.FrameRate = max(1/delay - 10, 20);
writerObj.FrameRate = 20;
open(writerObj);
for i = 1:length(F)
    frame = F(i);
    writeVideo(writerObj,frame);
end
close(writerObj);
%


function [more,network,parents] = explore(network,counter,obstacles,step,parents,counter_parent)
    pRan = [30*rand,30*rand]; % random point within limits
    index = -1 ; dist = inf;
    for i = 1:length(network(:,1)) % find closest node in given tree
       d = sqrt((pRan(1) - network(i,1))^2  + (pRan(2) - network(i,2))^2 );
       if (dist > d)
            dist = d;
            index = i;
       end
    end
    counter_index = -1; more = 1;
    if (dist >= step) % atleast delta dist away
        theta = atan2(pRan(2) - network(index,2),pRan(1) - network(index,1)); 
        pNew = [network(index,1) + step*cos(theta) , network(index,2) + step*sin(theta)]; %  node in direction of new_point delta distance away
        collision = false; 
        for i = 1:length(obstacles(:,1)) % checking collision with obstacles
            if (sqrt((pNew(1) - obstacles(i,1))^2 + (pNew(2) - obstacles(i,2))^2) <= obstacles(i,3))
                collision = true;
            end
        end
        
        if (~collision) % not colliding
            network = [network;pNew]; % add to network
            parents = [parents;[parents(end,1)+1 , index]]; %add its index and parent index
            for i = 1:length(counter(:,1)) % closest node in counter tree 
                if (sqrt((pNew(1) - counter(i,1))^2 + (pNew(2) - counter(i,2))^2) <= step)
                    more = 0;
                    counter_index = i;
                    break;
                end
            end
        end
        if (more == 0) % if counter tree node closer than step distance, plot green edges
            plot([pNew(1),counter(counter_index,1)],[pNew(2),counter(counter_index,2)],'g') % new edge
            i_n = length(parents(:,1)); i_n_parent = parents(i_n,2);
            while(i_n ~= i_n_parent)
                plot([network(i_n,1),network(i_n_parent,1)],[network(i_n,2),network(i_n_parent,2)],'g'); % iterate to parent, plot green
                i_n = i_n_parent;
                i_n_parent = parents(i_n,2);
            end
            i_c = counter_index ; i_c_parent = counter_parent(i_c,2);
            while(i_c ~= i_c_parent)
                plot([counter(i_c,1),counter(i_c_parent,1)],[counter(i_c,2),counter(i_c_parent,2)],'g'); % same step for counter tree
                i_c = i_c_parent;
                i_c_parent = counter_parent(i_c,2);
            end
        end
        
    end
    
end