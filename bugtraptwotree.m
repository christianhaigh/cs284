%% Code setup (Do not modify, but please read) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles. Each obstacle is a cell in Obs. An obstacle is
% represented as a convex hull of a number of points. These points are
% stored in the cells of Obs.
% First row is x, second is y (position of vertices)
w = 0.5;
Obs{1} = [0 0;5 0;5 w;0 w]';
Obs{2} = [0 0;2*w 0;w 10;0 10]';
Obs{3} = [0 10-w;5 10;5 10+w;0 10+w]';
Obs{4} = [5-w 0;5+w 0;5+w 5;5 5]';
Obs{5} = [5-w 10+w;5+w 10+w;5+w 6.25;5 6.25]';
Obs{6} = [4 5;5+w 5;5+w 5+w;4 5+w]';
Obs{7} = [4 6.25;5+w 6.25;5+w 6.25+w;4 6.25+w]';

% Bounds on world
world_bounds_x = [-8,10];
world_bounds_y = [-4,14];

% Draw obstacles
figure(1); clf; hold on;
axis([world_bounds_x world_bounds_y]);

for k = 1:length(Obs)
    patch(Obs{k}(1,:),Obs{k}(2,:),'r');
end

% Start and goal positions
xy_start = [4;1]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
xy_goal = [-4;6]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10);

% Initialize RRT. The RRT will be represented as a 2 x N list of points. So
% each column represents a vertex of the tree.

% LINE 1
% Initialize Ta
rrt_verts_a = zeros(2,1000);
rrt_verts_a(:,1) = xy_start;
% Initialize Tb
rrt_verts_b = zeros(2,1000);
rrt_verts_b(:,1) = xy_goal;
N_a = 1;
N_b = 1;
nearGoal = false; % This will be set to true if goal has been reached
minDistGoal = 0.25; % This is the convergence criterion. We will declare
                    % success when the tree reaches within 0.25 in distance
                    % from the goal. DO NOT MODIFY.

% Extension parameter
d = 0.5; % This controls how far the RRT extends in each step. DO NOT MODIFY.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RRT algorithm
while ~nearGoal

    % LINE 14 
    if N_b < N_a 
        rrt_verts_temp = rrt_verts_a;
        rrt_verts_a = rrt_verts_b;
        rrt_verts_b = rrt_verts_temp;
        N_temp = N_a; 
        N_a = N_b; 
        N_b = N_temp;
    end

    % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal. This promotes movement to the
    % goal.

    if rnd < 0.05 && isequal(rrt_verts_a(:,1),xy_start)
        xy = xy_goal;
    elseif rnd < 0.05 && isequal(rrt_verts_a(:,1),xy_goal)
        xy = xy_start;
    else
        %% FILL ME IN
        % Sample (uniformly) from space (with probability 0.95). The space is defined
        % with the bounds world_bounds_x and world_bounds_y defined above.
        % So, the x coordinate should be sampled in the interval
        % world_bounds_x and the y coordinate from world_bounds_y.
        x = (world_bounds_x(2)-world_bounds_x(1)).*rand(1) + world_bounds_x(1);
        y = (world_bounds_y(2)-world_bounds_y(1)).*rand(1) + world_bounds_y(1);
        xy = [x;y]; % Should be a 2 x 1 vector
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Check if sample is collision free
    collFree = isCollisionFree(Obs,xy); % Write this function. 
                               % Your code from part (a) will be useful here.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                        
                                        
    % If it's not collision free, continue with loop
    if ~collFree
        continue;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, find closest point in existing tree. 
    % The points in the existing tree are rrt_verts_a(:,1:N)
    closest_vert = closestVertex(rrt_verts_a(:,1:N_a),xy); % Write this function
    % Extend tree towards xy from closest_vert. Use the extension parameter
    % d defined above as your step size. In other words, the Euclidean
    % distance between new_vert and closest_vert should be d (do not modify
    % d. It should be 0.5).
    vector = xy - closest_vert;
    if norm(vector) > d
        new_vert = vector/norm(vector)*d + closest_vert;
    else 
        new_vert = xy; 
    end
    
    % Check if new_vert is collision free
    collFree = isCollisionFree(Obs,new_vert); % Same function you wrote before.

    % If it is not collision free, continue with loop
     if ~collFree
        continue;
     end
     
    % Plot extension (Comment the next 3 lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    figure(1)
    plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
       
    N_a = N_a +1;
    if N_a > size(rrt_verts_a,2)
        rrt_verts_a = [rrt_verts_a zeros(size(rrt_verts_a))];
    end
    new_vert_a = new_vert;
    rrt_verts_a(:,N_a) = new_vert;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    closest_vert = closestVertex(rrt_verts_b(:,1:N_b),new_vert);
    vector = new_vert - closest_vert;
    
    if norm(vector) > d
        new_vert = vector/norm(vector)*d + closest_vert;
    end   

    collFree = isCollisionFree(Obs,new_vert);
    if ~collFree
        continue;
    end
 
    N_b = N_b+1;
    if N_b > size(rrt_verts_b,2)
        rrt_verts_b = [rrt_verts_b zeros(size(rrt_verts_b))];
    end
    rrt_verts_b(:,N_b) = new_vert;
    
    % Check if we have reached goal
    if new_vert == new_vert_a
        break;
    end

     % Plot extension (Comment the next 3 lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    figure(1)
    plot(new_vert(1),new_vert(2),'ro','MarkerFaceColor','y','MarkerSize',5);
    line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% Plot vertices in RRT
plot(rrt_verts_a(1,1:N_a),rrt_verts_a(2,1:N_a),'bo','MarkerFaceColor','b','MarkerSize',5);
plot(rrt_verts_b(1,1:N_b),rrt_verts_b(2,1:N_b),'bo','MarkerFaceColor','b','MarkerSize',5);
disp(N_a + N_b);

