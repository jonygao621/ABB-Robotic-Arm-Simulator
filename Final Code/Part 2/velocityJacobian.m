%   velocityJacobian returns the velocity jacobian of the manipulator
%   given an array of links created by the createLink function and the 
%   current joint variables. 
% 
%   Jv = the velocity jacobian
%   JvDot = the time derivative of the velocity jacobian
%   
%   linkList = the list of joint parameters created with createLink
%   paramList = the current theta or d values for the joints. (an Nx1 array)
%   paramRateList = the current theta_dot and d_dot values for the joints. 
%   (an Nx1 array)
%
%   If paramRateList is not provided (check with 
%   exist(‘paramRateList’,’var’) ), then return [] for JvDot. Otherwise 
%   calculate JvDot as well.
%
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018
%
function [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)   
    % Vel Jac Function:
    % Number of Joints
    Link_qty = length(linkList);

    % Pre allocate a list of values that need to be stored in memory between
    % loops
    zlast = zeros(3,6);
    woi = zeros(3,6);
    doi = zeros(3,6); 
    doi_dot = zeros(3,6); 
    Hi = zeros(4,6*4);
%     list = repmat(struct( 'zlast', zeros(3,1),...   % Z i-1, rotation axis for link i in base frame
%                           'Woi', zeros(3,1),...     % Angular velocity of origin i in base frame
%                           'doi', zeros(3,1),...     % Position of Origin i relative to base in base frame
%                           'doi_dot', zeros(3,1),... % Velocity of Origin i relative to base in base frame
%                           'Hi', eye(4),...          % Transformation matrix from Origin i-1 to i in i-1 frame
%                           'Fi', zeros(3,1),...      % Inertial Force on link i in base frame
%                           'Ni', zeros(3,1) ),...    % Inertial Torque on link i in base frame
%                   Link_qty,1);

    % Initialize link variables that get propagated forward
    T0i = eye(4); % Transform from 0 to joint i

    V = zeros(3,1); % Velocity in base frame
    W0 = zeros(3,1); % Angluar Velocity in base joint frame
    W = W0; % W will be used as the variable in the remaining code

    for i=1:Link_qty % begin forward iteration from base to tool

        L_i = linkList(i);
        % Calculate link transform from i-1 to i
        H = dhFwdKine(L_i, paramList(i));

        % extract distance from joint 0 to joint i-1
        d_0_im1 = T0i(1:3,4);
        z_0_im1 = T0i(1:3,3);

        % Transformation & Rotation matrix from joint 0 to i
        T0i = T0i*H;

        % extract distance from joint i-1 to joint i in base frame view
        d_0_i = T0i(1:3,4);
        d_im1_i = d_0_i-d_0_im1;

        % Update joint velocity, acceleartion, angular Acceleration, and
        % angular velocity in world frame.
        % Rotational Joint
            % Joint i angular velocity in base frame:
            W = W + paramRateList(i)*z_0_im1;

            % Joint i linear velocity in base frame:
            V = V + cross(W,d_im1_i);


        % Save values specific to calculating Jv and JvDot:
        doi(:,i) = d_0_i; % save d_0_i from base to joint i in base frame view
        woi(:,i) = W - W0; % Save W in joint i in base frame view without initial conditions 
        doi_dot(:,i) = V - cross(W0, d_0_i); % Save V in joint i in base frame view without initial conditions
        zlast(:,i) = z_0_im1; % Save z_0_im1 vector
        Hi(:,(4*i-3):4*i) = H; % Transformation matrix from i-1 to i

    end % End forward iterations

    % Initialize variables for calculating Jv and JvDot:
    Jv = zeros(6, Link_qty);
    JvDot = zeros(6, Link_qty);
    d0N = doi(:,end); % Extract Distance from Base to end effector in base frame
    V0N = doi_dot(:,end); % Extract Velocity of end effector in base frame

    % Calculate Jv and JvDot:
        for i = Link_qty:-1:1 % From last joint to base
            % Displacement and velocity differences without initial
            % conditions from frame i to end effector in base frame:
            if i > 1
                d_iN = d0N - doi(:,i-1);
                V_iN = V0N - doi_dot(:,i-1);
            else
                d_iN = d0N;
                V_iN = V0N;
            end

            % The following if-statement performs the following:
            % - Populates each column of the jacobian by joint type (end to base).
            % Rotational Joint
                % Populate Jv and JvDot: 
                Jv(1:3,i) = cross(zlast(:,i),d_iN);
                Jv(4:6,i) = zlast(:,i);

                JvDot(1:3,i) = cross(cross(woi(:,i), zlast(:,i)), d_iN)+...
                               cross(zlast(:,i), V_iN);
                JvDot(4:6,i) = cross(woi(:,i),zlast(:,i));
        end
end