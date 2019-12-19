%   cpMap returns the matrix packing of the cross product operator. 
%   Ie. Given vectors W and V, cross(W) * V = W x V
% 
%   X = cpMap(w) takes a 3x1 vector w and outputs w into a 3x3 matrix that
%   can be multiplied by another 3x1 vector v to give the cross product of
%   w x v, or cross(w)*v.
%
%   X = The resulting 3x1 vector w in 3x3 matrix form so that it can easily
%   be multipled by another 3x1 vector v to give the cross product of
%   w x v, or cross(w)*v.
%   
%   w = The input 3x1 vector that will be put into 3x3 matrix for so it can
%   be multiplied by another 3x1 vector v to give the cross product of 
%   w x v, or cross(w)*v. 
%   
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

%k represented as a 3x3 matrix for Cross Products
function X = cpMap(w)
    X = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0;];
end