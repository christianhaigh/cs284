function [utraj,xtraj]=Dircol(obj,x0,xf)
  tf0 = 0.1;
  N = 21;
  traj_opt = DircolTrajectoryOptimization(obj,N,[2 6]);
  traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
  traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);
  traj_opt = traj_opt.addRunningCost(@cost);
  traj_opt = traj_opt.addFinalCost(@finalCost);
  traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
end