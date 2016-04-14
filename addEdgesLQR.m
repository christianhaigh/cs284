function new_verts = addEdgesLQR(closest_vertices,xy, K)
    k_verts = size(closest_vertices);
    new_verts = repmat([0;0;0],1,k_verts)
    function xdot = fdynamics(t,x,g,b,u)
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
    for k = 1:k_verts(2)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Prep
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        % Need to change all this
        while closest_vertices(1,k) > 3*pi/2
            closest_vertices(1,k) = closest_vertices(1,k) - 2*pi; 
        end  
        while closest_vertices(1,k) < -pi/2 
            closest_vertices(1,k) = closest_vertices(1,k) + 2*pi;
        end 
        if closest_vertices(1,k) > xy(1)
            wrap = 2*pi;
        else 
            wrap = -2*pi;
        end 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Closest Vertex to New Vertex
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if closest_vertices(3,k) == 0
            distance = closest_vertices(1:2,k) - xy; 
            while distance(1) > 2*pi
                distance(1) - 2*pi; 
            end
            u = -K*(distance) + u0;

            if u > 5 
                u = 5;
            elseif u < -5
                u = -5;
            end 

            time = [0,0.1];
            [ts,xs] = ode45(@ode_dynamics,time,closest_vertices(1:2,k));
            new_vert = [xs(length(xs),:)';0];
            while(new_vert(1) > 3*pi/2)
                new_vert(1) = new_vert(1) - 2*pi; 
            end
            while(new_vert(1) < -pi/2)
                new_vert(1) = new_vert(1) + 2*pi;
            end 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % New Vertex to Closest Vertex
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif closest_vertices(3,k) == 1
            distance = xy - closest_vertices(1:2,k); 
            while distance(1) > 2*pi
                distance(1) - 2*pi; 
            end
            u = -K*(distance) + u0;

            if u > 5 
                u = 5;
            elseif u < -5
                u = -5;
            end 
            
            time = [0,0.1];
            [ts,xs] = ode45(@ode_dynamics,time,xy);
            new_vert = [xs(length(xs),:)';1];
            while(new_vert(1) > 3*pi/2)
                new_vert(1) = new_vert(1) - 2*pi; 
            end
            while(new_vert(1) < -pi/2)
                new_vert(1) = new_vert(1) + 2*pi;
            end 
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Now Include it
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        new_verts(:,k) = new_vert;
    end 
end 