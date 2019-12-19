%   angleAxis2Rot returns the rotation matrix encoded by a rotation of 
%   theta radians about the unit vector k axis..
% 
%   R = angleAxis2Rot(k, theta) takes in a rotation theta about an axis k 
%   and returns a 3D rotation matrix R that corresonds to that rotation 
%   theta about that axis k.
%
%   R = 3D rotation matrix that corresponds to the rotation theta about
%   axis k.
%
%   k = The axis that theta is rotating around that corresponds to the
%   rotation matrix R.
%   theta = The rotation about the axis k that corresponds to the
%   rotation matrix R.
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018
%
function R = angleAxis2Rot(k, theta)
    c_theta = cos(theta);
    s_theta = sin(theta);
    
    v_theta = 1-c_theta;
    r11 = k(1)^2*v_theta+c_theta;
    r12 = k(1)*k(2)*v_theta-k(3)*s_theta;
    r13 = k(1)*k(3)*v_theta+k(2)*s_theta;
    r21 = k(1)*k(2)*v_theta+k(3)*s_theta;
    r22 = k(2)^2*v_theta+c_theta;
    r23 = k(2)*k(3)*v_theta-k(1)*s_theta;
    r31 = k(1)*k(3)*v_theta-k(2)*s_theta;
    r32 = k(2)*k(3)*v_theta+k(1)*s_theta;
    r33 = k(3)^2*v_theta+c_theta;
    R = [r11, r12, r13; r21, r22, r23; r31, r32, r33];
end