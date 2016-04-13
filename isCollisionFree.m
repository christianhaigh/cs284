function collFree = isCollisionFree(Obs,xy)
	function colls = collisionCheck(obs,points)
	    xys = points';
	    obs_xys = obs'; 
	    ixs = convhull(obs_xys);
	    chull = obs_xys(ixs',:); 
	    xq = xys(:,1); 
	    yq = xys(:,2); 
	    xv = chull(:,1); 
	    yv = chull(:,2); 
	    colls = ~inpolygon(xq, yq, xv, yv);
	end
	dim = size(xy');
	collFree = ones(1, dim(1));
	for k = 1:length(Obs)
		collFree = bsxfun(@and,collisionCheck(Obs{k}, xy)', collFree);
	end 
end 