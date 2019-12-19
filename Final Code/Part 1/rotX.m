% rotX Returns a rotation matrix describing a rotation about the X axis 
% (theta in radians).
% 
%   R = rotX(theta) takes any rotation theta about the x-axis and outputs a 3D
%   rotation matrix R.
%
%   R = 3D rotation matrix R
%   
%   theta = rotation theta about the x-axis
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function R = rotX(theta)
    R = [1,0,0; 0, cos(theta), -sin(theta); 0, sin(theta), cos(theta)];
end