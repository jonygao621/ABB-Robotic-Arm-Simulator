%   transform2Twist returns the twist vector corresponding to the provided 
%   homogenous transform matrix. The twist should be stacked [v; w*th].
% 
%   t = transform2Twist(H) takes a homogeneous transformation matrix 'H' 
%   that corresponds to that twist vector 't', where t = [v; w*th], and 
%   outputs that twist vector t
%
%   t = the twist vector 't', where t = [v; w*th],that corresponds to the
%   homogeneous transformation matrix 'H'.
%
%   H = the homogeneous transformation matrix 'H' that corresponds to
%   that twist vector 't', where t = [v; w*th].
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   9/30/2018

function t = transform2Twist(H)
     R=[H(1,1), H(1,2), H(1,3); H(2,1), H(2,2), H(2,3); H(3,1), H(3,2), H(3,3)];
     d=[H(1,4); H(2,4); H(3,4)];
     [k, theta] = rot2AngleAxis(R);
    % Special cases occurs when theta = n*pi where n = 0,1,2,...etc.
     if abs(theta) < 0.0001 || abs(theta-2*pi) < 0.0001
            w = [1; 0; 0]; % If there is zero rotation, then w can equal any vector
            ohm = w*theta;
            v = d;
     elseif abs(theta-pi) < 0.001
            w1 = sqrt((1+R(1,1))/2);
            w2 = sqrt((1+R(2,2))/2);
            w3 = sqrt((1+R(3,3))/2);
            % We get two values for w because sqrt gives that + & -, but both 
            % give the same R
            if abs(w1) > 0.0001
                w2 = (2*w1).\R(2,1);
                w3 = (2*w1).\R(3,1);
            elseif abs(w2) > 0.0001
                w1 = (2*w2).\R(2,1);
                w3 = (2*w2).\R(3,2);
            else
                w1 = (2*w3).\R(3,1);
                w2 = (2*w3).\R(3,2);
            end
            w = [w1;w2;w3];
            ohm = w*theta;
            v = d;
     else
         w1 = (R(3,2)-R(2,3))/(2*sin(theta));
         w2 = (R(1,3)-R(3,1))/(2*sin(theta));
         w3 = (R(2,1)-R(1,2))/(2*sin(theta));
         w = [w1; w2; w3];
         wT = w.';
         ohm = w*theta;
         I = [1, 0, 0; 0, 1, 0; 0, 0, 1];
         coss_w = cpMap(w);
         v = sin(theta)/(2*(1-cos(theta)))*I*d+((2*(1-cos(theta))-theta*sin(theta))/(2*theta*(1-cos(theta))))*w*wT*d-(1/2)*coss_w*d;
     end

    % Exponential Rotation Matrix
    % v_theta=1-cos(theta);
    % e_wtheta=[w1^2*v_theta+cos(theta), w1*w2*v_theta-w3*sin(theta), w1*w3*v_theta+w2*sin(theta);
    %         w1*w2*v_theta+w3*sin(theta), w2^2*v_theta+cos(theta), w2*w3*v_theta-w1*sin(theta);
    %         w1*w3*v_theta-w2*sin(theta), w2*w3*v_theta+w1*sin(theta), w3^2*v_theta+cos(theta)];
    % Verify R = e_wtheta
    % error=R-e_wtheta;

    t = [v; ohm];
end