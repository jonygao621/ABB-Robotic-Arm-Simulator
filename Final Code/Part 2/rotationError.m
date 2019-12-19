%   rotationError returns an angle*axis vector, expressed in the
%   reference frame, describing what rotation is necessary to take 
%   Rot_current to Rot_desired.  
% 
%   rot_error_vector = an angle*axis vector, expressed in the reference 
%   frame, describing what rotation is necessary to take Rot_current to 
%   Rot_desired. It is a 3x1 vector describing the axis of rotation 
%   multiplied by the angle of rotation (in radians) necessary to transform 
%   Rot_current into Rot_desired. See angleAxis2Rot for more info.
%   
%   Rot_desired = the rotation matrix describing the desired coordinate 
%   frame (of the robot for example) in the reference frame (of the world
%   for example).
%   Rot_current is the rotation matrix describing the current coordinate 
%   frame (of the robot for example) in the reference frame (of the world
%   for example).
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018
%
function rot_error_vector = rotationError(Rot_desired, Rot_current) 
    
    R_error = Rot_desired*Rot_current';
    [ k, theta ] = rot2AngleAxis(R_error);
    
    rot_error_vector = theta*k;
end