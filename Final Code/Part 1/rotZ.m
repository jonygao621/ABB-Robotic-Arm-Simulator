% rotZ Returns a rotation matrix describing a rotation about the Z axis 
% (theta in radians).
%
%   R = rotZ(theta) takes any rotation theta about the z-axis and outputs a 3D
%   rotation matrix R.
%
%   R = 3D rotation matrix R resulting from rotation about z-axis
%   
%   theta = rotation theta about the z-axis
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function R = rotZ(theta)
    R = [cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
end