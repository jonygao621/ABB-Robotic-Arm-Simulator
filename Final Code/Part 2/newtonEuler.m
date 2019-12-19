%   newtonEuler uses the Newton Euler method to calculate the manipulator
%   dynamics of each link from base frame to tool frame. It then calculates
%   the joint torques in reverse order.
%
% Output(s):
%   jointTorques = [Nx1] array of joint torques in N*m from inverse dynamics.
%   Jv = [6xN] velocity jacobian achieved by forward kinematics.
%   JvDot = [6xN] time derivative of velocity jacobian.
%
% Input(s):
%   linklist = [Nx1] array of structs from createLink.m function, with members:
%       a           = [meters]  DH parameter a
%       d           = [meters]  DH parameter d
%       alpha       = [radians] DH parameter alpha
%       theta       = [radians] DH parameter theta
%       mass        = [kg]      link mass
%       inertia     = [kg*m^2]  link mass moment of inertia
%       com         = [meters]  position of the link’s center of mass
%       isRotary    = [Boolean] true: rotary joint; false: prismatic joint.
%
%	paramList = [Nx1] array of current joint angles/distances.
%       At the tool distal end, there should be no joint parameters.
%
%	paramListDot = [Nx1] array of current joint angle/distance speeds.
%
%	paramListDDot = [Nx1] array of current joint angle/distance accelerations
%
%	boundry_conditions = a structure with members:
%       base_angular_velocity       = [rad/s] 
%       base_angular_acceleration   = [rad/s^2]
%       base_linear_acceleration	= [m/s^2] with gravity as necessary
%       distal_force                = [N]   at tool distal end.
%       distal_torque               = [N*m] at tool distal end.
%
% Created by:
%   Casey Duncan
%   10834922
%   MEGN 544
%   11/19/2018

function jointTorques = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundry_conditions)

% Number of Joints
Link_qty = length(linkList);

% Pre allocate a list of values that need to be stored in memory between
% loops
list = repmat(struct( 'zlast', zeros(3,1),...   % Z i-1, rotation axis for link i in base frame
                      'Woi', zeros(3,1),...     % Angular velocity of origin i in base frame
                      'doi', zeros(3,1),...     % Position of Origin i relative to base in base frame
                      'doi_dot', zeros(3,1),... % Velocity of Origin i relative to base in base frame
                      'Hi', eye(4),...          % Transformation matrix from Origin i-1 to i in i-1 frame
                      'Fi', zeros(3,1),...      % Inertial Force on link i in base frame
                      'Ni', zeros(3,1) ),...    % Inertial Torque on link i in base frame
              Link_qty,1);

% Initialize link variables that get propagated forward
T0i = eye(4); % Transform from 0 to joint i

V = zeros(3,1); % Velocity in base frame
W0 = boundry_conditions.base_angular_velocity; % Angluar Velocity in joint frame
W = W0; % W will be used as the variable in the remaining code

Wdot = boundry_conditions.base_angular_acceleration; % Angular Acceleration in joint frame
Vdot = boundry_conditions.base_linear_acceleration; % Linear acceleration in joint frame
    
for i=1:Link_qty % begin forward iteration from base to tool

    L_i = linkList(i);
    % Calculate link transform from i-1 to i
    H = dhFwdKine(L_i, paramList(i));
    R_im1_i = H(1:3,1:3);

    % extract distance from joint 0 to joint i-1
    d_0_im1 = T0i(1:3,4);
    z_0_im1 = T0i(1:3,3);

    % Transformation & Rotation matrix from joint 0 to i
    T0i = T0i*H;
    R0i = T0i(1:3,1:3);

    % extract distance from joint i-1 to joint i in base frame view
    d_0_i = T0i(1:3,4);
    d_im1_i = d_0_i-d_0_im1;

    % Center of Mass (COM) distance with respect to base frame from
    % origin i to the COM of link i:
    r0_com_i = R0i*(R_im1_i'*H(1:3,4)+L_i.com);

    % Update joint velocity, acceleartion, angular Acceleration, and
    % angular velocity in world frame.
    if L_i.isRotary

        % Joint i angular acceleration and velocity in base frame:
        Wdot = Wdot+paramListDDot(i)*z_0_im1+paramListDot(i)*cross(W,z_0_im1);
        W = W + paramListDot(i)*z_0_im1;

        % CoM linear acceleration from i-1 to i, in base frame:
        r_dd_i = Vdot+cross(Wdot,r0_com_i)+cross(W,cross(W, r0_com_i));

        % Joint i linear acceleration and velocity in base frame:
        Vdot = Vdot + cross(Wdot,d_im1_i) + cross(W,cross(W,d_im1_i));
        V = V + cross(W,d_im1_i);

    else % It is prismatic, where angular motion remains constant:

        % CoM linear acceleration from joint i-1 to i, in base frame:
        r_dd_i = Vdot+cross(Wdot,r0_com_i)+cross(W,cross(W, r0_com_i))+...
                 paramListDDot(i)*z_0_im1+2*paramListDot(i)*cross(W,z_0_im1);

        % Joint i linear acceleration and velocity in base frame:
        Vdot = Vdot + cross(Wdot,d_im1_i) + cross(W, cross(W,d_im1_i))+...
               paramListDDot(i)*z_0_im1+2*paramListDot(i)*cross(W,z_0_im1);
        V = V+cross(W,d_im1_i)+paramListDot(i)*z_0_im1;  
    end

    % Calculate and save Inertial Force and Torque in i'th frame:
    r_dd_0i = R0i'*r_dd_i;
    W_i = R0i'*W;
    Wdot_i = R0i'*Wdot;
    F = L_i.mass*r_dd_0i;
    n = L_i.inertia*(Wdot_i)+cross(W_i,L_i.inertia*W_i);

    % Save values specific to calculating Jv and JvDot:
    list(i).doi = d_0_i; % save d_0_i from base to joint i in base frame view
    list(i).woi = W - W0; % Save W in joint i in base frame view without initial conditions 
    list(i).doi_dot = V - cross(W0, d_0_i); % Save V in joint i in base frame view without initial conditions
    list(i).zlast = z_0_im1; % Save z_0_im1 vector

    % Save values specific to calculating joint forces / torques:
    list(i).Hi = H; % Transformation matrix from i-1 to i
    list(i).Fi = F; % Inertial Force in i frame
    list(i).Ni = n; % Inertial Torque in i frame

end % End forward iterations

% Initialize variables for calculating Jv and JvDot:
Jv = zeros(6, Link_qty);
JvDot = zeros(6, Link_qty);
d0N = list(end).doi; % Extract Distance from Base to end effector in base frame
V0N = list(end).doi_dot; % Extract Velocity of end effector in base frame

% Initialize and preallocate variables for force/torque propagation:
F0 = boundry_conditions.distal_force; % 3-D joint force
F = F0;
N0 = boundry_conditions.distal_torque; % 3-D joint torque
N = N0;
jointTorques = zeros(Link_qty, 1); % Z-component of joint forces/torques

% Calculate Jv and JvDot:
    for i = Link_qty:-1:1 % From last joint to base

        % Extract rotation matrix to convert i+1 to i frame vectors:
        if i == Link_qty
            R_i_ip1 = eye(3); % No rotation occurs beyond end effector end
        else
            R_i_ip1 = list(i+1).Hi(1:3,1:3); % R_i_ip1 rotation is applied
        end
        
        R_im1_i = list(i).Hi(1:3,1:3); % R_im1_i rotation from i-1 to i frame

        % Update Force on joint i in i frame view:
        F = list(i).Fi + R_i_ip1*F;

        % Update Torque on joint i in i frame view:
        N = list(i).Ni + R_i_ip1*N + cross(R_im1_i'*list(i).Hi(1:3,4), F ) + ...
            cross(linkList(i).com, list(i).Fi);
        
        % Displacement and velocity differences without initial
        % conditions from frame i to end effector in base frame:
        if i > 1
            d_iN = d0N - list(i-1).doi;
            V_iN = V0N - list(i-1).doi_dot;
        else
            d_iN = d0N;
            V_iN = V0N;
        end

        % The following if-statement performs two tasks:
        % - Populates joint force or torque in i-1 frame by Z-component.
        % - Populates each column of the jacobian by joint type (end to base).
        if linkList(i).isRotary % Rotational Joint
            
            % Joint i torque is the Z component in i-1 frame:
            jointTorques(i,1) = dot([0;0;1],R_im1_i*N); 

            % Populate Jv and JvDot: 
            Jv(1:3,i) = cross(list(i).zlast,d_iN);
            Jv(4:6,i) = list(i).zlast;

            JvDot(1:3,i) = cross(cross(list(i).woi, list(i).zlast), d_iN)+...
                           cross(list(i).zlast, V_iN);
            JvDot(4:6,i) = cross(list(i).woi,list(i).zlast);

        else % Prismatic
            % Joint i force is the Z component in i-1 frame:
            jointTorques(i,1) = dot([0;0;1], R_im1_i*F); 

            % Populate Jv:
            Jv(1:3,i) = list(i).zlast;
            Jv(4:6,i) = zeros(3,1);

            % Populate JvDot:
            JvDot(1:3,i) = cross(list(i).woi, list(i).zlast);
            JvDot(4:6,i) = zeros(3,1);
        end
    end
end