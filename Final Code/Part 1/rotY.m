% rotY Returns a rotation matrix describing a rotation about the Y axis 
% (theta in radians).
%
%   R = rotY(theta) takes any rotation theta about the y-axis and outputs a 3D
%   rotation matrix R.
%
%   R = 3D rotation matrix R
%   
%   theta = rotation theta about the y-axis
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function R = rotY(theta)
    R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end