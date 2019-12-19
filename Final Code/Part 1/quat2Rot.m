%   quat2Rot returns the rotation matrix that corresponds to the 
%   quaternion, stacked [q0;q_vec].
% 
%   R = quat2Rot(Q) takes a quaternion Q = [qo;q_vec] and outputs the 3D 
%   rotation matrix R that corresonds to that quaternion Q.
%
%   R = the 3D rotation matrix R that corresonds to that quaternion 
%   rotation Q = [qo;q_vec].
%
%   Q = the quaternion rotation Q = [qo;q_vec] that corresponds to the 3D
%   rotation matrix R
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function R = quat2Rot(Q)
    q0 = Q(1);
    q = Q(2:4);
    R = (q0^2-(q.')*q)*eye(3)+2*q0*cpMap(q)+2*(q*q.');
end