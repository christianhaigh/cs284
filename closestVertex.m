function closest_vert = closestVertex(tree_verts,xy)
	closest_vert = tree_verts(:,1);
	distance = norm(closest_vert - xy);
	tree_length = size(tree_verts);
	for k = 1:tree_length(2)
		new_dist = norm(tree_verts(:,k) - xy);
		if new_dist < distance
			distance = new_dist; 
			closest_vert = tree_verts(:,k);
		end 
	end
end 