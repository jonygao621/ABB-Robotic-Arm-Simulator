% rot2RPY Returns the roll, pitch and yaw corresponding to a given rotation 
% matrix. It should return the two valid solutions corresponding to the 
% +sqrt and –sqrt. Each output is then a [2x1] vector with the plus 
% solution on top.
% 
%   [roll, pitch, yaw] = rot2RPY(R) takes a rotation matrix R and outputs a
%   rotation 'roll' about the z-axis, rotation 'pitch' about the y-axis, 
%   and rotation 'yaw' about the x-axis, where the rotations are performed
%   in the order of roll, pitch, yaw.
%
%   roll = The resulting rotation 'roll' about the z-axis of the three 
%   rotations roll, pitch and yaw, where the rotations are performed in 
%   the order of roll, pitch, yaw.
%   pitch = The resulting rotation 'pitch' about the y-axis of the three 
%   rotations roll, pitch and yaw, where the rotations are performed in 
%   the order of roll, pitch, yaw.
%   yaw = The resulting rotation 'yaw' about the x-axis of the three 
%   rotations roll, pitch and yaw, where the rotations are performed in 
%   the order of roll, pitch, yaw.

%   R = 3D rotation matrix R resultiing from combination of rotations about 
%   the combination of rotations roll, pitch, and yaw
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function [roll, pitch, yaw] = rot2RPY(R)
    pitch1 = atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2));
    pitch2 = atan2(-R(3,1), -sqrt(R(1,1)^2+R(2,1)^2));
    pitch = [pitch1; pitch2];
    % Special cases occurs when theta = (2*n-1)*pi/2 where n = 0,1,2,...etc.
    if abs(pitch(1)-pi/2) < 0.0001 || abs(pitch(2)-pi/2) < 0.0001
        roll = atan2(R(1,2),R(2,2));
        roll = roll*ones(2,1);
        yaw = zeros(2,1);
    elseif abs(pitch(1)+pi/2) < 0.0001 || abs(pitch(2)+pi/2) < 0.0001
        roll = atan2(-R(1,2),R(2,2));
        roll = roll*ones(2,1);
        yaw = zeros(2,1);
    else
        yaw = atan2(cos(pitch).\R(2,1),cos(pitch).\R(1,1));
        roll = atan2(cos(pitch).\R(3,2),cos(pitch).\R(3,3));
    end
end