function [closest_vert, K, ix] = closestVertexLQR(tree_verts,xy)
	g = 9.81; 
	b = 0.1;
	distance = inf;
	closest_vert = tree_verts(1:2,1);
	lower_bound = -pi/2;
	upper_bound = 3*pi/2;
	while xy(1) > 3*pi/2
		xy(1) = xy(1) - 2*pi; 
	end  
	while xy(1) < -pi/2 
		xy(1) = xy(1) + 2*pi;
	end 
	R = 0.1; 
	Q = eye(2);
	A = [0,1;-g*cos(xy(1)),-b];
	B = [0;1];
	[K,S] = lqr(A,B,Q,R);
	tree_length = size(tree_verts);
	ix = 1;
	for k = 1:tree_length(2)
		new_vert = tree_verts(1:2,k);
		if new_vert(1) > xy(1)
			wrap = 2*pi;
		else 
			wrap = -2*pi;
		end 
		new_vert_a = new_vert; 
		new_vert_b = [new_vert(1)-wrap;new_vert(2)];
		if (new_vert_a-xy)'*S*(new_vert_a-xy) < (new_vert_b-xy)'*S*(new_vert_b-xy)
			dist = (new_vert_a-xy)'*S*(new_vert_a-xy);
			new_vert = new_vert_a;
		else 
			dist = (new_vert_b-xy)'*S*(new_vert_b-xy);
			new_vert = new_vert_b;
		end 
		if dist < distance  
			distance = dist;
			closest_vert = new_vert;
			ix = k;
		end 
	end 
	while(closest_vert(1) > 3*pi/2)
		closest_vert(1) = closest_vert(1) - 2*pi
	end
	while(closest_vert(1) < -pi/2)
		closest_vert(1) = closest_vert(1) + 2*pi
	end
end 