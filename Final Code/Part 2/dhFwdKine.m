%   dhFwdKine returns the forward kinematics of a manipulator with the
%   provided DH parameter set. 
% 
%   H = forward kinematics of a manipulator with the provided DH parameter 
%   set.
%   
%   linkList = an array of links, each created by createLink
%   paramList = an array containing the current state of their joint 
%   variables.
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018
%
function H = dhFwdKine(linkList, paramList) 
    Link_qty = length(linkList);
    T = eye(4);
    
    for i = 1:Link_qty
        L_i = linkList(i);
        
        % If link is Rotary, use theta from paramList:
        th_i = paramList(i);
        T_i = dhTransform(L_i.a, L_i.d, L_i.alpha, th_i);
        
        
        % Calculate transformation matrix (T_0,N) for manipulator from 
        % world frame to end effector
        T = T*T_i;
    end
    
    % The forward kinematics for the manipulator is just 
    H = T;
end