%   rot2AngleAxis returns the angle and axis corresponding to a rotation 
%   matrix.
% 
%   [k, theta] = rot2AngleAxis(R) takes in a 3D rotation matrix R and
%   returns the angle (theta) and axis (k) corresponding to a rotation 
%   matrix.
%
%   k = The axis that theta is rotating around that corresponds to the
%   rotation matrix R.
%   theta = The rotation about the axis k that corresponds to the
%   rotation matrix R.
%   
%   R = 3D rotation matrix that corresponds to the rotation theta about
%   axis k.
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function [k, theta] = rot2AngleAxis(R)
    cos_th = (trace(R)-1)/2;
    sin_th = sqrt((R(3,2)-R(2,3))^2+(R(1,3)-R(3,1))^2+(R(2,1)-R(1,2))^2)/2;
    theta = atan2(sin_th,cos_th);
    % Special cases occurs when theta = n*pi where n = 0,1,2,...etc.
    if abs(theta) < 0.0001 || abs(theta-2*pi) < 0.0001
        k = [1; 0; 0]; % If there is zero rotation, then k can equal any vector
    elseif abs(theta-pi) < 0.001
        k1 = sqrt((1+R(1,1))/2);
        k2 = sqrt((1+R(2,2))/2);
        k3 = sqrt((1+R(3,3))/2);
        % We get two values for k because sqrt gives that + & -, but both 
        % give the same R
        if abs(k1) > 0.0001
            k2 = (4*k1).\(R(1,2)+R(2,1));
            k3 = (4*k1).\(R(1,3)+R(3,1));
        elseif abs(k2) > 0.0001
            k1 = (4*k2).\(R(1,2)+R(2,1));
            k3 = (4*k2).\(R(3,2)+R(2,3));
        else
            k1 = (4*k3).\(R(1,3)+R(3,1));
            k2 = (4*k3).\(R(3,2)+R(2,3));
        end
        k = [k1;k2;k3];
    else
        k = (1/(2*sin_th))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    end
end