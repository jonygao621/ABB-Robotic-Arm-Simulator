%   transError returns a 6x1 vector, where the first 3 elements 
%   are position error (desired - current), and the last three elements are 
%   an angle-axis representation of rotation error. Both expressed in the 
%   shared base frame.  
% 
%   error_vector = a 6x1 vector describing the error 
%   ([pos_error;rot_error]) as expressed in the shared base frame.
%   
%   Td = the homogenious matrix describing the desired coordinate pose (of 
%   the robot for example) in the reference frame (of the world for 
%   example).
%   Tc = the homogenious matrix describing the current coordinate frame 
%   (of the robot for example) in the reference frame (of the world for 
%   example).
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018
%
function [error_vector] = transError(Td, Tc) 
    Rot_d = Td(1:3,1:3);
    d_d = Td(1:3,4);
    Rot_c = Tc(1:3,1:3);
    d_c = Tc(1:3,4);
    
    pos_error = d_d-d_c;
    rot_error = rotationError(Rot_d, Rot_c);
    error_vector = [pos_error; rot_error]; 
end