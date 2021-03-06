function new_vert = extendLQR(closest_vert,xy, K)
	u0 = 0;
	g = 9.81;
	b = 0.1;

    lower_bound = -pi/2;
    upper_bound = 3*pi/2;
    while xy(1) > 3*pi/2
        xy(1) = xy(1) - 2*pi; 
    end  
    while xy(1) < -pi/2 
        xy(1) = xy(1) + 2*pi;
    end 
    while closest_vert(1) > 3*pi/2
        closest_vert(1) = closest_vert(1) - 2*pi; 
    end  
    while closest_vert(1) < -pi/2 
        closest_vert(1) = closest_vert(1) + 2*pi;
    end 
    if closest_vert(1) > xy(1)
        wrap = 2*pi;
    else 
        wrap = -2*pi;
    end 
    distance = closest_vert - xy; 
    while distance(1) > 2*pi
        distance(1) - 2*pi; 
    end
    u = -K*(distance) + u0;

    if u > 10 
        u = 10;
    elseif u < -10
        u = -10;
    end 
    
    function xdot = fdynamics(t,x,g,b,u)
        dtheta = x(2);
        distance = x - xy;
        u = -K*(distance) + u0;
        if u > 10 
            u = 10;
        elseif u <-10 
            u = -10; 
        end 
        ddtheta = (u - g*sin(x(1)) - b*x(2));
        xdot = [dtheta; ddtheta];
    end 

    function xdot = ode_dynamics(t, x)
        xdot = fdynamics(t,x,g,b,u);
    end 
    time = [0,0.5];
    [ts,xs] = ode45(@ode_dynamics,time,closest_vert);
    new_vert = xs(length(xs),:)';
    dist = norm(new_vert - xy);
    for k=1:length(xs)
        lqr_dist = norm(xs(k,:)' - xy);
        if lqr_dist < dist
            new_vert = xs(k,:)';
            dist = lqr_dist;
        end 
    end 
    while(new_vert(1) > 3*pi/2)
        new_vert(1) = new_vert(1) - 2*pi; 
    end
    while(new_vert(1) < -pi/2)
        new_vert(1) = new_vert(1) + 2*pi;
    end 
end 