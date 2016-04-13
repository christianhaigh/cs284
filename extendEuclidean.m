function new_vert = extendEuclidean(closest_vert,xy)
	us = linspace(-5,5,20);
	g = 9.81;
	b = 0.1;

	function ddtheta = dynamics(g,b,th,thd,u, t)
      ddtheta = (u - g*sin(th) - b*thd);
    end

    function xdot = fdynamics(t,x,g,b,u)
        theta = x(1);
        dtheta = x(2);
        if u > 5 
            u = 5
        elseif u <-5 
            u = -5 
        end 
        ddtheta = (u - g*sin(x(1)) - b*x(2));
        xdot = [dtheta; ddtheta];
    end 

    function xdot = ode_dynamics(t, x)
        xdot = fdynamics(t,x,g,b,u);
    end 

    distance = 0; 
    u = 100;
    new_vert = [];

	ulength = size(us);
	for k = 1:ulength(2)
    	ddtheta = dynamics(g,b,closest_vert(1), closest_vert(2), us(k), 0.1);
        u_vector = [closest_vert(2); ddtheta]/norm([closest_vert(2); ddtheta]);
        dist = dot(u_vector, xy);
    	if dist > distance 
    		distance = dist; 
    		u = us(k); 
    	end 
    end 
    time = [0,0.1];
    [ts,xs] = ode45(@ode_dynamics,time,closest_vert);
    new_vert = xs(length(xs),:)';
    if new_vert(1) > 3*pi/2 
        new_vert(1) = new_vert(1) - 2*pi 
    elseif new_vert(1) < -pi/2
        new_vert(1) = new_vert(1) + 2*pi
    end 
end 