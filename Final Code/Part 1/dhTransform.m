%   dhTransform returns the homogenous transform corresponding to the 
%   provide DH parameters for a link.
% 
%   H = dhTransform(a, d, alpha, theta) takes in DH parameters 
%   (a, d, alpha, theta) that correspond to a given link and outputs a 
%   homogenous transform 'H' that corresponds to those DH parameters.
%
%   H = the homogenous transform 'H' that corresponds to the DH parameters 
%   (a, d, alpha, theta) for a given link
%
%   a = the translation from coordinate frame z_i-1 to z_i about the x_i
%   axis.
%   d = the translation from coordinate frame x_i-1 to x_i about the z_i-1
%   axis.
%   alpha = the rotation from coordinate frame z_i-1 to z_i about the x_i
%   axis.
%   theta = the rotation from coordinate frame x_i-1 to x_i about the z_i-1
%   axis.
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function H = dhTransform(a, d, alpha, theta)
    zero_v = [0,0,0];
    
    trans_z = [1,0,0,0;0,1,0,0;0,0,1,d;0,0,0,1];
    rot_z = [rotZ(theta), zero_v.'; zero_v, 1];
    trans_x = [1,0,0,a;0,1,0,0;0,0,1,0;0,0,0,1];
    rot_x = [rotX(alpha), zero_v.'; zero_v, 1];
    H = trans_z*rot_z*trans_x*rot_x;
end