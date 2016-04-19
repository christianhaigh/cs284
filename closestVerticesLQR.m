function [closest_vertices] = closestVerticesLQR(prm_verts, S_verts, K_verts, K)
	closest_vertices = zeros(K,length(prm_verts));
	distances = inf(K,length(prm_verts));
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Find distance from point 1 to point 2
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for k = 1:length(prm_verts)
		vert_1 = prm_verts(:,k);
		for j = 1:length(prm_verts)
			if j ~= k
				vert_2 = prm_verts(:,j);
				S = S_verts(:,j);
				K = K_verts(:,j);
				if vert_1(1) > vert_2(1)
					wrap = 2*pi;
				else 
					wrap = -2*pi;
				end 
				vert_1_a = vert_1; 
				vert_1_b = [vert_1(1)-wrap;vert_1(2)];
				if (vert_1_a-vert_2)'*S*(vert_1_a-vert_2) < (vert_1_b-vert_2)'*S*(vert_1_b-vert_2)
					dist = (vert_1_a-vert_2)'*S*(vert_1_a-vert_2);
				else 
					dist = (vert_1_b-vert_2)'*S*(vert_1_b-vert_2);
				end 
				[argvalue, argmax] = max(distances(:,k));
				if norm(dist) < argvalue 
					distances(argmax,k) = norm(dist);
					closest_vertices(argmax,k) = j;
				end 
			end
		end 
	end 
end