%   twist2Transform returns the homogeneous transformation matrix 
%   corresponding to a 6 element twist vector. The twist should be 
%   stacked [v; w*th].
% 
%   H = twist2Transform(t) takes a twist vector 't', where t = [v; w*th],
%   and outputs a homogeneous transformation matrix 'H' that corresponds to
%   that twist vector.
%
%   H = the homogeneous transformation matrix 'H' that corresponds to
%   that twist vector 't', where t = [v; w*th].
%
%   t = the twist vector 't', where t = [v; w*th],that corresponds to the
%   homogeneous transformation matrix 'H'.
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function H = twist2Transform(t)
    v1=t(1,1);
    v2=t(2,1);
    v3=t(3,1);
    v=[v1; v2; v3];
    Om1=t(4,1);
    Om2=t(5,1);
    Om3=t(6,1);
    Omega=[Om1; Om2; Om3];

    theta=norm(Omega);
    w_hat=Omega/theta;
    wT_hat=w_hat.';

    I=[1, 0, 0; 0, 1, 0; 0, 0, 1];
    zero=[0, 0, 0];

    %Exponential Rotation Matrix
    R=cos(theta)*I+sin(theta)*cpMap(w_hat)+(1-cos(theta))*w_hat*wT_hat;

    %Translation Matrix
    d=(I-R)*cpMap(w_hat)*v+theta*w_hat*wT_hat*v;

    %Homogeneous Transformation Matrix
    H = [R, d; zero, 1];
end