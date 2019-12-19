%   abbInvKine Returns the joint angles required to reach the desired 
%   transformation for the ABB arm we solved in homework. Make sure you 
%   handle the degeneracy gracefully as discussed in class!
% 
%   th1 – th6 = the 6 joint angles. These are real scalar values if th_last 
%   is provided and are 8x1 vectors if th_last is not provided.
%   reachable = is true if the transform can be achieved. Is false if it 
%   cannot be achieved. 
%
%   T_des = the desired homogeneous transform
%   th_last = a 6x1 vector of the last set of thetas used to select a 
%   specific solution. If th_last is not provided (check with 
%   exist(‘th_last’,’var’) ), then provide all 8 possibilities, otherwise 
%   return the closest solution to th_last. These should all be real values!.
%
%   In class, we have measured the parameters to be:
%   a = [0; 0.270; 0.070; 0; 0; 0]
%   d = [0.290; 0; 0; 0.302; 0; 0.072]
%   alph = [-pi/2; 0; -pi/2; pi/2; -pi/2]
%   th_offset = [0; pi/2; 0; 0; 0]
%   
%   The encoder for Joint 1 is shifted to prevent the arm from being in the
%   table in zero angle reading. To convert use the following equation:
%   th_ABB = th_DH + pi/2
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018

function [th1,th2,th3,th4,th5,th6, reachable] = abbInvKine(T_des, th_last)
    a = [0; 0.270; 0.070; 0; 0; 0];
    d = [0.290; 0; 0; 0.302; 0; 0.072];
    alph = [-pi/2; 0; -pi/2; pi/2; -pi/2];
    th_offset = [0; pi/2; 0; 0; 0];
    
    R_06 = T_des(1:3,1:3);
    d_0_06 = T_des(1:3,4);
    d05 = d_0_06-d(6)*R_06*[0;0;1];
    
    th1 = zeros(8,1);
    th2 = zeros(8,1);
    th3 = zeros(8,1);
    th4 = zeros(8,1);
    th5 = zeros(8,1);
    th6 = zeros(8,1);
    for i=1:8
        %find th1 (8 solutions...)
        if i == 1 || i == 2 || i == 3 || i == 4
            th1(i) = wrapToPi(atan2(d05(2),d05(1)));
        else
            th1(i) = wrapToPi(atan2(d05(2),d05(1))+pi);
        end

        T01 = dhTransform(a(1),d(1),alph(1),th1(i));

        d15 = T01(1:3,1:3)'*(d05-d(1)*[0;0;1]);

        % solve for th3
        r = sqrt(a(3)^2+d(4)^2);
        nu = atan2(d(4),a(3));

        mag_d15 = sqrt(d15(1,1)^2+d15(2,1)^2);
        if isreal(mag_d15) %Reachable: This happens when any of the sqrt outputs imaginary values
            reachable = true;
        else
            reachable = false;
        end

        cosPhi = (a(2)^2+r^2-norm(d15)^2)/(2*a(2)*r);  
        half_angle = (1-cosPhi)/(1+cosPhi);
        sqrt_h_a = sqrt(half_angle);
        if isreal(sqrt_h_a)==0 || reachable == 0
            reachable = false;
        else
            reachable = true;
        end

        if i == 1 || i == 3 || i == 5 || i == 7
            th3(i) = pi+2*atan(sqrt_h_a)-nu;
            th3(i) = wrapToPi(real(th3(i)));
        else
            th3(i) = pi+2*atan(-sqrt_h_a)-nu;
            th3(i) = wrapToPi(real(th3(i)));
        end

        % solve for th2
        alpha = a(3)*sin(th3(i))+d(4)*cos(th3(i));
        beta = a(2)+a(3)*cos(th3(i))-d(4)*sin(th3(i));
        
        M = [-alpha beta; beta alpha];
        sin_cos_th2 = M\[d15(1);d15(2)];
        sin_th2 = sin_cos_th2(1);
        cos_th2 = sin_cos_th2(2);
        th2(i) = wrapToPi(real(atan2(sin_th2,cos_th2)));

        % Use Rotation to get th4 - th6 
        R03 = rotZ(th1(i))*rotX(alph(1))*rotZ(th2(i))*rotX(alph(2))*rotZ(th3(i))*rotX(alph(3));
        R36 = R03'*T_des(1:3,1:3);
        
        cos_th5 = R36(3,3);
        sin_th5 = sqrt(R36(3,1)^2 + R36(3,2)^2);
        if isreal(sin_th5)==0 || reachable == 0
            reachable = false;
        else
            reachable = true;
        end

        if i==1 || i==2 || i==5 || i==6
            th5(i) = atan2(sin_th5,cos_th5);
        else
            th5(i) = atan2(-sin_th5,cos_th5);
        end
        th5(i) = real(th5(i));

        % Degeneracy
        if abs(th5(i))<1E-10 || abs(abs(th5(i))-pi)<1E-10
            if R36(3,3)>0
                if (th_last(4)+th_last(6))>pi
                    th_sum = atan2(R36(2,1),R36(1,1))+2*pi;
                elseif (th_last(4)+th_last(6))<-pi
                    th_sum = atan2(R36(2,1),R36(1,1))-2*pi;
                else
                    th_sum = atan2(R36(2,1),R36(1,1));
                end
                coef = [2,0,1;0,2,1;1,1,0];
                t_last = [2*th_last(4);2*th_last(6);th_sum];
            else
                if (th_last(4)-th_last(6))>pi
                    th_sum = atan2(-R36(2,1),-R36(1,1))+2*pi;
                elseif (th_last(4)-th_last(6))<-pi
                    th_sum = atan2(-R36(2,1),-R36(1,1))-2*pi;
                else
                    th_sum = atan2(-R36(2,1),-R36(1,1));
                end
                coef = [2,0,1;0,-2,1;1,-1,0];
                t_last = [2*th_last(4);-2*th_last(6);th_sum];
            end
            t_4_6_lam = coef\t_last;
            th4(i) = t_4_6_lam(1);
            th6(i) = t_4_6_lam(2);
        else
            th4(i) = wrapToPi(atan2(-R36(2,3)/sin(th5(i)),-R36(1,3)/sin(th5(i))));
            th6(i) = wrapToPi(atan2(-R36(3,2)/sin(th5(i)),R36(3,1)/sin(th5(i))));
        end   
    end
    
    %% Output: Tyler Evans helped me with this portion
    theta = [th1 th2 th3 th4 th5 th6];
    if exist('th_last','var')
        %This section is used to calculate wind-up
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        int = (th_last(5))/pi;
        int_whole = floor(int);
        rem = int - int_whole;
        if rem > .25 && rem < 0.75
            dec_add = 0.5;
        else
            dec_add = 0;
        end
        quad = int_whole*pi+dec_add;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        th_last = wrapToPi(th_last);

        for i = 1:6
            if i == 2
                Err(:,i) = abs(wrapToPi((th_last(i)-th_offset(i)))-theta(:,i));
            else
                Err(:,i) = abs(th_last(i)-theta(:,i));
            end
        end

    for i = 1:8
        norm_err(i,1) = norm(Err(i,:));
    end
    
    [~, II] = min(norm_err);
    Small = Err < 0.2;
    check=all(Small,2);
    
    if check(II)
        I = II;
    else
        [~,I] = max(check);
    end
       
    th1 = wrapToPi(theta(I,1));
    th2 = wrapToPi(theta(I,2)+pi/2);
    th3 = wrapToPi(theta(I,3));
    th4 = wrapToPi(theta(I,4));
    th5 = wrapToPi(theta(I,5));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if abs(int)>1
        add = quad-th5;
        th5 = add+th5;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    th6 = wrapToPi(theta(I,6));
else
    th1 = wrapToPi(theta(:,1));
    th2 = wrapToPi(theta(:,2)+pi/2);
    th3 = wrapToPi(theta(:,3));
    th4 = wrapToPi(theta(:,4));
    th5 = wrapToPi(theta(:,5));
    th6 = wrapToPi(theta(:,6));
end
end