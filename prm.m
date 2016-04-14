%% Code setup (Do not modify, but please read) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Bounds on world
world_bounds_th = [-pi/2,(3/2)*pi];
world_bounds_thdot = [-10,10];

xs = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
ys = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);


% Start and goal positions
figure(1); clf;
xy_start = [xs;ys]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);

% Initialize PRM. The PRM will be represented as a 2 x N list of points, and the connections
% between vertices will be represented as a N x N matrix of edges. Finally, the distances 
% between vertices will be represented as a N x N matrix of distances. 
no_nodes = 10;
prm_verts = zeros(2,no_nodes);
prm_graph = zeros(no_nodes);
prm_verts(:,1) = xy_start;

N = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The number of closest vertices considered for each new point
k_verts = 5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1); hold on;
axis([world_bounds_th, world_bounds_thdot]);
hxy = plot(0,0,'ro');
% PRM algorithm
while N < no_nodes;
    % Sample point
    rnd = rand(1);
    % With probability 0.05, sample a random point from another tree. 
    if rnd < 0.05
        xy = xy_goal;
    else
        % Sample from space with probability 0.95
        xs = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
        ys = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);
        xy = [xs;ys];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Extend closest vertex in direction of random point
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [closest_vert,K,ix] = closestVertexLQR(prm_verts(:,1:N),xy);
    % This will be the new vertex added to the graph
    new_vert = extendLQR(closest_vert,xy,K); 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, add it to tree    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    N = N+1;
    if N > size(prm_verts,2)
        prm_verts = [prm_verts zeros(size(prm_verts))];
    end
    prm_verts(:,N) = new_vert;
    
    disp(N);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Extend closest k - 1 vertices to the newly formed vertex if they can get there
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [closest_vertices,K,ix] = closestVerticesLQR(prm_verts(:,1:N),new_vert,k_verts-1);
    % Now we need to dircol to the new vert TODO 
    new_verts = addEdgesLQR(closest_vertices,new_vert,K); 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Now we check if those vertices are near the closest vertices and form links if yes
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for k = 1:k_verts 
        if norm(new_verts(1:2,k)-new_vert) < 0.25
            prm_graph = addConnection(prm_graph,prm_verts,new_verts(:,k),new_vert)
        end 
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
       
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