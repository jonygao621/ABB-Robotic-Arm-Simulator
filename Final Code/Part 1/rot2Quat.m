%   rot2Quat returns the quaternion [qo;q_vec] that corresponds to the 
%   rotation matrix.
% 
%   Q = rot2Quat(R) takes a 3D rotation matrix R that corresonds to that
%   quaternion Q = [qo;q_vec] and outputs the quaternion rotation Q.
%
%   Q = the quaternion rotation Q = [qo;q_vec] that corresponds to the 3D
%   rotation matrix R
%
%   R = the 3D rotation matrix R that corresonds to that quaternion 
%   rotation Q = [qo;q_vec].
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function Q = rot2Quat(R)
    q_0 = sqrt(1+trace(R))/2;
    % Special case occurs when q0 = 0
    if abs(q_0) < 0.0001
        q1=sqrt(-(R(2,2)+R(3,3))/2);
        q2=sqrt(-(R(1,1)+R(3,3))/2);
        q3=sqrt(-(R(1,1)+R(2,2))/2);
        if abs(q1) > 0.0001
            q2 = R(1,2)/(2*q1);
            q3 = R(3,1)/(2*q1);
        elseif abs(q2) > 0.0001
            q1 = R(1,2)/(2*q2);
            q3 = R(2,3)/(2*q2);
        else
            q1 = R(3,1)/(2*q3);
            q2 = R(2,3)/(2*q3);
        end
    else
    q1 = (R(3,2)-R(2,3))/(4*q_0);
    q2 = (R(1,3)-R(3,1))/(4*q_0);
    q3 = (R(2,1)-R(1,2))/(4*q_0);
    end
    q = [q1;q2;q3];
    Q = [q_0; q];
end