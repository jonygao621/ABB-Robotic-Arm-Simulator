%   constAccelInterp provides the position (p), velocity (v), and
%   acceleration (a) at time t for a trajectory interpolated using the 
%   constant acceleration approach. Each of these are length M vectors.
% 
%   p = the position at time t for a trajectory interpolated using the 
%   constant acceleration approach.
%   v = the velocity at time t for a trajectory interpolated using the 
%   constant acceleration approach.
%   a = the acceleration at time t for a trajectory interpolated using the 
%   constant acceleration approach.
%   
%   t = the time of interest (these will be evenly spaced)
%   trajectory = a Nx(M+1) array of points. There are N waypoints in the 
%   trajectory of dimension M. The first column is the time for each point 
%   in the trajectory and the remaining M columns are the point to be 
%   reached at that time. 
%   transPercent = The percentage of the trajectory time to use for the 
%   constant acceleration transition. This must be in the range [0, 0.5]
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018

function [p, v, a] = constAccelInterp(t, trajectory, transPercent)
    t_traj = trajectory(:,1);
    pos = trajectory(:,2:end);
    
%     t_traj = [t_traj(1,:); t_traj; t_traj(end,:)];
%     pos = [pos(1,:); pos; pos(end,:)];
    
     %Find time of the next waypoint
    n = find(t_traj > t,1);
    
    if (n > 2 && t < (t_traj(n-1)*(1-transPercent)+t_traj(n)*(transPercent)))
        % Still finishing previous acceleration
        p = pos(n-1,:) - (pos(n-1,:)-pos(n-2,:))/(4*transPercent*(t_traj(n-1)-t_traj(n-2)))*(t-t_traj(n-1)-transPercent)^2 ...
            + (pos(n,:)-pos(n-1,:))/(4*transPercent*(t_traj(n)-t_traj(n-1)))*(t-t_traj(n-1)+transPercent)^2;
        v = - (pos(n-1,:)-pos(n-2,:))/(2*transPercent*(t_traj(n-1)-t_traj(n-2)))*(t-t_traj(n-1)-transPercent) ...
            + (pos(n,:)-pos(n-1,:))/(2*transPercent*(t_traj(n)-t_traj(n-1)))*(t-t_traj(n-1)+transPercent);
        a = - (pos(n-1,:)-pos(n-2,:))/(2*transPercent*(t_traj(n-1)-t_traj(n-2))) ...
            + (pos(n,:)-pos(n-1,:))/(2*transPercent*(t_traj(n)-t_traj(n-1)));
    
    elseif (t > (t_traj(n-1)) && t < (t_traj(n)-transPercent))
        % Entering next acceleration
        p = pos(n,:) - (pos(n,:)-pos(n-1,:))/(4*transPercent*(t_traj(n)-t_traj(n-1)))*(t-t_traj(n)-transPercent)^2 ...
            + (pos(n+1,:)-pos(n,:))/(4*transPercent*(t_traj(n+1)-t_traj(n)))*(t-t_traj(n)+transPercent)^2;
        v = - (pos(n,:)-pos(n-1,:))/(2*transPercent*(t_traj(n)-t_traj(n-1)))*(t-t_traj(n)-transPercent) ...
            + (pos(n+1,:)-pos(n,:))/(2*transPercent*(t_traj(n+1)-t_traj(n)))*(t-t_traj(n)+transPercent);
        a = - (pos(n,:)-pos(n-1,:))/(2*transPercent*(t_traj(n)-t_traj(n-1))) ...
            + (pos(n+1,:)-pos(n,:))/(2*transPercent*(t_traj(n+1)-t_traj(n)));
    else
%     elseif (t < (t_traj(n-1)*(transPercent)+t_traj(n)*(1-transPercent)))
        % Constant velocity
        p = pos(n,:) - (t_traj(n)-t)/(t_traj(n)-t_traj(n-1))*(pos(n,:)-pos(n-1,:));
        v = (pos(n,:)-pos(n-1,:))/(t_traj(n)-t_traj(n-1));
        a = v*0;
    end
end

