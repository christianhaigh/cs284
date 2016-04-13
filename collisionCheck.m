function colls = collisionCheck(points,obs)
    xys = points';
    obs_xys = obs'; 
    ixs = convhull(obs_xys);
    chull = obs_xys(ixs',:); 
    xq = xys(:,1); 
    yq = xys(:,2); 
    xv = chull(:,1); 
    yv = chull(:,2); 
    colls = inpolygon(xq, yq, xv, yv);
end