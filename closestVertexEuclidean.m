function [closest_vert, ix] = closestVertexEuclidean(tree_verts,xy)
	closest_vert = tree_verts(1:2,1);
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
	ix = 1;
	distance = min(norm(closest_vert - xy),norm([closest_vert(1)-wrap;closest_vert(2)] - xy));
	tree_length = size(tree_verts);
	for k = 1:tree_length(2)
		new_vert = tree_verts(1:2,k);
		if new_vert(1) > xy(1)
			wrap = 2*pi;
		else 
			wrap = -2*pi;
		end 
		% need to put while loops here
		new_dist = min(norm(new_vert - xy),norm([new_vert(1)-wrap;new_vert(2)] - xy));
		if new_dist < distance
			distance = new_dist; 
			closest_vert = new_vert;
			ix = k;
		end 
	end
end 