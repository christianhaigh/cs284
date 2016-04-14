function [closest_vertices, K, ix] = closestVerticesLQR(tree_verts,xy,k_verts)
	g = 9.81; 
	b = 0.1;
	distances = repmat(inf,1,k_verts);
	closest_vertices = repmat([tree_verts(1:2,1);0],1,k_verts);
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
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Check for shortest distances from graph to new point
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Change the vertices if the new distances are smaller 
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		dist_diff = 0
		for l = 1:k_verts
			dist_check = distances(l) - dist
			if dist_check > dist_diff
				dist_diff = dist_check
				vert_change = l
			end 
		end 
		if dist_diff > 0
			distances(vert_change) = distances(vert_change) - dist_diff
			closest_vertices(:,vert_change) = [new_vert;0]
		end 
	end 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Check for shortest distances from new point to graph
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for k = 1:tree_length(2)
		new_vert = tree_verts(1:2,k);
		if new_vert(1) > xy(1)
			wrap = 2*pi;
		else 
			wrap = -2*pi;
		end 
		new_vert_a = new_vert; 
		new_vert_b = [new_vert(1)-wrap;new_vert(2)];
		if (xy - new_vert_a)'*S*(xy - new_vert_a) < (xy - new_vert_b)'*S*(xy - new_vert_b)
			dist = (xy - new_vert_a)'*S*(xy - new_vert_a);
			new_vert = new_vert_a;
		else 
			dist = (xy -  new_vert_b)'*S*(xy - new_vert_b);
			new_vert = new_vert_b;
		end 
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		% Change the vertices if the new distances are smaller 
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		dist_diff = 0
		for l = 1:k_verts
			dist_check = distances(l) - dist
			if dist_check > dist_diff
				dist_diff = dist_check
				vert_change = l
			end 
		end 
		if dist_diff > 0
			distances(vert_change) = distances(vert_change) - dist_diff
			closest_vertices(:,vert_change) = [new_vert;1]
		end 
	end 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% make sure that all the closest vertices are within the frame
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for l = 1:k_verts
		while(closest_vertices(1,l) > 3*pi/2)
			closest_vertices(1,l) = closest_vertices(1,l) - 2*pi
		end
		while(closest_vertices(1,l) < -pi/2)
			closest_vertices(1,l) = closest_vertices(1,l) + 2*pi
		end
	end
end 