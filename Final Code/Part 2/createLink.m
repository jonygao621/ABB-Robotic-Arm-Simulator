%   createLink creates a structure with the following members:
% 
%   L = structure containing a, d, alpha, theta, centOfMass, mass, inertia
%   
%   a = DH parameter a (meters)
%   d = DH parameter d (meters)
%   alpha = DH parameter alpha (radians),
%   theta –= DH parameter theta (radians)
%   mass = link mass (kg)
%   inertia = link mass moment of inertia (kg m^2)
%   centOfMass = the position of the link’s center of mass
%   isRotary = Boolean true if it is a rotary joint false if it is a 
%   prismatic joint.
%
%   All vectors and tensors are to be expressed in the Link’s coordinate 
%   frame
%
%   On call pass an empty array ([]) to the joint variable that changes, 
%   i.e., for a rotary joint a call would be: createLink(.14, 3.4, pi/3, 
%   [],…
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018
%
function L = createLink(a, d, alpha, theta, centOfMass, mass, inertia) 
    if isempty(theta)
        isRotary = true;
    else
        isRotary = false;
    end
        
    L = struct('a',a, 'd',d, 'alpha',alpha, 'theta',theta, ...
               'com',centOfMass, 'mass',mass, 'inertia',inertia, ...
               'isRotary',isRotary);
end