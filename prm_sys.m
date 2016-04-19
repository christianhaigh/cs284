%% Code setup (Do not modify, but please read) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Bounds on world
world_bounds_th = [-pi/2,(3/2)*pi];
world_bounds_thdot = [-10,10];

% Start and goal positions
figure(1); clf;
xs_start = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
ys_start = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);
xs_goal = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
ys_goal = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);

% plot start and goal positions
xy_start = [xs_start;ys_start]; plot(xy_start(1),xy_start(2),'ro','MarkerFaceColor','r','MarkerSize',10); hold on;
xy_goal = [xs_goal;ys_goal]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10); drawnow;

% Initialize n random vertices in state space
n = 100;
xs = (world_bounds_th(2) - world_bounds_th(1))*rand(1,n) + world_bounds_th(1);
ys = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1,n) + world_bounds_thdot(1);
prm_verts = [xs;ys];
hold on;
plot(prm_verts(1,:),prm_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
drawnow;
N = 1;


% LQR Analysis get S and K for each vertex
g = 9.81; 
b = 0.1;
R = 0.1; 
Q = eye(2);
S_verts = []
K_verts = []
for k = 1:length(prm_verts)
    A = [0,1;-g*cos(prm_verts(1,k)),-b];
    B = [0;1];
    [K,S] = lqr(A,B,Q,R);
    S_verts = [S_verts, S];
    K_verts = [K_verts, K];
end 

% K Closest neighbors 
k = 5
[closest_vertices] = closestVerticesLQR(prm_verts, S_verts, K_verts, k);


figure(1); hold on;
axis([world_bounds_th, world_bounds_thdot]);

for k = 1:length(prm_verts)
    ix = closest_vertices(1,k);
    K = [K_verts(:,ix*2-1),K_verts(:,ix*2)];
    new_vert = extendLQR(prm_verts(:,ix),prm_verts(:,k),K); 
    disp(prm_verts(:,ix));
    disp(new_vert);
end



%{
    % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal. This promotes movement to the
    % goal.
    if rnd < 0.05
        xy = xy_goal;
    else
        % Sample from space with probability 0.95
        xs = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
        ys = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);
        xy = [xs;ys];
    end
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(method, 'euclidean')
        [closest_vert,ix] = closestVertexEuclidean(rrt_verts(:,1:N),xy); % Write this function
    elseif strcmp(method, 'lqr')
        [closest_vert,K,ix] = closestVertexLQR(rrt_verts(:,1:N),xy); % Write this function
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(method, 'euclidean')
        new_vert = extendEuclidean(closest_vert,xy); % Write this function
    else
        
    end

    delete(hxy);
    figure(1);
    hxy = plot(xy(1),xy(2),'r.');axis([world_bounds_th, world_bounds_thdot]); 

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
     
    % Plot extension (Comment the next few lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    figure(1)
    hold on
    plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    % Plot line (but only if we are not wrapping to the other side of the
    % plot)
    if abs(closest_vert(1) - new_vert(1)) < 0.75*(2*pi)
        line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
    end
    axis([world_bounds_th, world_bounds_thdot]);

    
    %% DO NOT MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, add it to tree    
    N = N+1;
    if N > size(rrt_verts,2)
        rrt_verts = [rrt_verts zeros(size(rrt_verts))];
    end
    rrt_verts(:,N) = [new_vert;ix];
    
    % Check if we have reached goal
    if norm(xy_goal-new_vert) < minDistGoal
        break;
    end
    disp(N);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
       
end

% Plot vertices in RRT
hold on;
plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
xys = []
while(N>0)
    N = rrt_verts(3,N);
    if N > 0
        disp(N);
        xys = [rrt_verts(1:2,N),xys];
    end 
end 
disp(xys);
ts = [];
for i=1:size(xys,2)
    ts = [ts, 0.1*(i-1)];
end 
xy_length = size(xys)
for k = 1:xy_length(2)-1
    first_vert = xys(:,k);
    second_vert = xys(:,k+1);
    disp(second_vert);
    if abs(first_vert(1) - second_vert(1)) < 0.75*(2*pi)
        line([first_vert(1),second_vert(1)],[first_vert(2),second_vert(2)],'Color','r');
    end
end
%disp(ts);
%pd = PendulumPlant;
%pv = PendulumVisualizer();
%traj = PPTrajectory(foh(ts,xys));
%traj = traj.setOutputFrame(pv.getInputFrame());
%playback(pv,traj);
%}