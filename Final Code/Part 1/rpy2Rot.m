% rpy2Rot returns a rotation matrix corresponding to a roll, pitch, yaw 
% encoded rotation. Note RPY is defined as the set of orthogonal rotations 
% rotZ(yaw)rotY(pitch)rotX(roll).
% 
%   R = rpy2Rot(roll,pitch,yaw) takes any combination of rotations about 
%   the z-axis, y-axis, and x-axis in that order, and outputs a 3D
%   rotation matrix R.
%
%   R = 3D rotation matrix R resultiing from combination of rotations about 
%   the z-axis, y-axis, and x-axis in that order
%   
%   roll = rotation psi about the x-axis
%   pitch = rotation theta about the y-axis
%   yaw = rotation phi about the z-axis
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function R = rpy2Rot(roll,pitch,yaw)
    R = rotZ(yaw)*rotY(pitch)*rotX(roll);
end