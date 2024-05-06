%% General Information:
%{ 
KR700 Simulation using the DH Apporach based on a simplified model.

Created as part of:
Malla, O. and Shanmugavel, M. (2024),
"Simplified model to study the kinematics of manipulators
with parallelogram linkages", Industrial Robot,
Vol. ahead-of-print No. ahead-of-print.
https://doi.org/10.1108/IR-01-2024-0046 

The code requires the use of Peter Corke's Robotics Toolbox for Matlab:
https://petercorke.com/toolboxes/robotics-toolbox/

Path Planning for the robot is also included.

%}
%% Notes:
% 1. theta2 and theta 3 ranges changes
% 2. Workspace is still not developed
% 3. Path Planning algorithms is not revised nor tested
%% Robot Specs (Dimensions are in mm):

L1 = 900; % To match the KR700 Studio L1 = 0.
L12 = 420;
L2 = 1300; % length of link 2
L3 = 1300; % length of link 3
% L4 = 22.0;  % length of link 4. L4 = 61 To match with RoboDk Model.
L4 = 300;
L5 = 300;

fprintf('KR700 : \n\n');
fprintf('Link lengths " mm ":\n %1.2f , %2.2f , %3.2f , %4.2f: \n \n', L1, L2, L3, L4);

%% NEED TO LOOK INTO THESE RANGES:
% Joints' limits:
%qmin =  [-165 , -40, -40 , -300];
qmin =  [-185 , -120, -10 , -350];
qmax =  [+185 , +10, +150, +350];

% Joint Limits Radians:
theta1_min = deg2rad(qmin(1)); % minimum joint angle 1
theta1_max = deg2rad(qmax(1)); % maximum joint angle 1
theta2_min = deg2rad(qmin(2)); % minimum joint angle 2
theta2_max = deg2rad(qmax(2)); % maximum joint angle 2
theta3_min = deg2rad(qmin(3)); % minimum joint angle 3
theta3_max = deg2rad(qmax(3)); % maximum joint angle 3
theta4_min = deg2rad(qmin(4)); % minimum joint angle 4
theta4_max = deg2rad(qmax(4)); % maximum joint angle 4

% Defining robot base relative to workspace origin point "Camera":
%Tbase1 = [1 0 0 0; 0 1 0 0; 0 0 1 -L1; 0 0 0 1];
Tbase1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
fprintf('Robot 1 base transformation from workspace base point: \n');
disp(Tbase1);

% Discretizing the joint rotations:
% SampleRate = 0.1; 
% n1 = ((qmax(1)-qmin(1))/SampleRate)+1; % 180/SampelRate = 1800 points
% n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
% n3 = ((qmax(3)-qmin(3))/SampleRate)+1;

% q1 = linspace(theta1_min,theta1_max,n1);
% q2 = linspace(theta2_min,theta2_max,n2);
% q3 = linspace(theta3_min,theta2_max,n3);

% DH Method with PSEUDO JOINT:
KR700M(1)= Link([0 , L1  , L12  , -pi/2]);
KR700M(2)= Link([0 ,  0  , L2 , 0    ]);
KR700M(3)= Link([0 ,  0  , L3 , 0    ]);
KR700M(4)= Link([0 ,  0  , L4 , -pi/2]);
KR700M(5)= Link([0 ,  L5 , 0  , 0    ]);

KR700M(1).qlim = [theta1_min,theta1_max];
%KR700M(2).offset = pi/2;
KR700M(2).qlim = [theta2_min,theta2_max];
%KR700M(3).offset = pi/2 - KR700M(2).theta;
KR700M(3).qlim = [-pi,pi/2];
KR700M(4).qlim = [-pi/2,pi]; % pseudo-joint
KR700M(5).qlim = [theta4_min,theta4_max];

Rob1 = SerialLink(KR700M,'name','KR700');

figure;
Q0 = [0,deg2rad(0),deg2rad(0),deg2rad(0),0];

Rob1.plot(Q0,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,0.6,0.2],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);
Rob1.teach;

%% Fkine:

Q = deg2rad(R2S([-165,-40,-20,-300]));
Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);

Rob1.fkine(Q)


%% Ikine:
% Q = deg2rad(R2S([100,50,50,0]));
% Rf = Rob1.fkine(Q);
% TP = Rf.t;

IntGuess = [0,0,0,0,0];
TP = [1500,1500,600]

% Call the inverse kinematics function (KR700Inverse) to compute Qsol
Ik = KR700Inverse(TP, IntGuess, true)
Rob1.fkine(Ik)
Rob1.plot(Ik);
rad2deg(Ik)
rad2deg(S2R(Ik))

%% Calculating the Workspace from the DH-Method:
% Calculating the Workspace: 

x   = 1;
% N   = 40;
N = 20;
q1 = (linspace(deg2rad(-90) ,deg2rad(90)    ,N))';
q2 = (linspace(deg2rad(0)   ,deg2rad(90)    ,N))';
q3 = (linspace(deg2rad(-10) ,deg2rad(90)    ,N))';
% For faster results q4 will be considered as 0
% q4 = (linspace(deg2rad(-90) ,deg2rad(90)    ,N))'; 
% TR1 = [zeros(4,4,N^4)];

TR1 = zeros(4,4,N^3);

for i = 1:length(q1)
    for j=1:length(q2)
        for ii = 1:length(q3)
            % for jj = 1:length(q4)  
               % TR1(:,:,x) = Rob1.fkine([q1(i),q2(j),q3(ii)-q2(j),-q3(ii),q4(jj)]);
               % for faster results q4 is considered 0 as it will not have
               % effects on the workspace with the system studied
               Q = [q1(i),q2(j),q3(ii)-q2(j),-q3(ii),0];
               TR1(:,:,x) = Rob1.fkine(Q);
               x = x+1;
            % end
        end
    end
end

A1 = transl(TR1);

% Centroid and Alpha Shape
% K-Means clsutering to find the center of mass for the points of Robot 2:
num_clusters1 = 1;
[~, centroid1] = kmeans(A1 , num_clusters1);

Shrink_Factor = 0.8;
[k_A,vol_A] = boundary(A1,Shrink_Factor);


%% Plotting the workspace of the Robot:

figure;
%Rob1.plot([0,0,0,0,0]);
%c = [0.2 0.2 0.6];
c=  [0.4 0.4 0.7];
% scatter3(A1(:,1),A1(:,2),A1(:,3),100,'filled','MarkerEdgeColor','b','MarkerFaceColor',c,'MarkerEdgeAlpha',0,'MarkerFaceAlpha',0.2)
scatter3(A1(:,1),A1(:,2),A1(:,3),'filled','CData', A1(:,2));
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of KR700 from the DH model: ");
axis equal;
hold off;

map = [0 0 0.2; 0 0 0.3; 0 0 0.4; 0 0 0.45; 0.1 0.1 0.5; 0.1 0.1 0.55; 
    0.15 0.15 0.6; 0.2 0.2 0.65; 0.25 0.25 0.7; 0.3 0.3 0.8; 0.4 0.4 0.9];

figure;
custom_colormap = map;

min_cdata = min(A1(:,1));
max_cdata = max(A1(:,1));
normalized_cdata = (A1(:,1) - min_cdata) / (max_cdata - min_cdata);
colormap(custom_colormap);
scatter3(A1(:,1), A1(:,2), A1(:,3), 'filled', 'CData', normalized_cdata,'MarkerEdgeAlpha',0.1,'MarkerFaceAlpha',0.4);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of KR700 from the DH model");
axis equal;
hold off;


%% Plotting the workspace of the Robot With AlphaShape:

figure;
trisurf(k_A, A1(:,1), A1(:,2), A1(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.7);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of KR700 *from DH model* with AlphaShape: Sf =  " + (Shrink_Factor));
axis equal;
hold off;


%% Create a Path from a set of desired wayPoints
% wayPoints = [1 0 0; 0 1 0; -1 0 0; 0 -1 0; 1 0 0];
% trajectory = cscvn(wayPoints');
% hold on
% fnplt(trajectory,'r',2);

%% Creating a Circular Path:

radius = 300;
nC = 50;
angles = linspace(0, 2*pi, nC);
wayPoints.X = 200 + (radius * cos(angles));
wayPoints.Y = radius * sin(angles);
wayPoints.Z = zeros(1,nC); % Assuming the circle lies in the xy plane (z = 0)

% Plotting the circular path
plot3(wayPoints.X, wayPoints.Y, wayPoints.Z, 'r.-', 'LineWidth', 2);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Circular Path through Waypoints');



%% Test Case Path Planning:

PathPlan(Rob1,wayPoints,false);

web('Path.gif');

%% FUNCTIONS:
% 1. Symbolic Equations for Geomtric and DH approach:

syms q1v q2v q3v q4v q5v L1v L2v L3v L4v L5v

TeqGeometric = trotz(q4v) * trotz(q1v) * troty(pi) * Tbase1;
display(simplify(TeqGeometric));

KR700M(1)= Link([0 , L1v  , 0  , -pi/2]);
KR700M(2)= Link([0 ,  0  , L2v , 0    ]);
KR700M(3)= Link([0 ,  0  , L3v , 0    ]);
KR700M(4)= Link([0 ,  0  , L4v , -pi/2]);
KR700M(5)= Link([0 ,  L5v , 0  , 0    ]);

qmin =  [-165 , -40, -20 , -300];
qmax =  [+165 , +85, +120, +300];

KR700M(1).qlim = [theta1_min,theta1_max];
KR700M(2).offset = -pi/2;
KR700M(2).qlim = [theta2_min,theta2_max];
KR700M(3).offset = pi/2 - KR700M(2).theta;
KR700M(3).qlim = [-pi,pi/2];
KR700M(4).qlim = [-pi/2,pi]; % pseudo-joint
KR700M(5).qlim = [theta4_min,theta4_max];

Robeq = SerialLink(KR700M,'name','KR700');
% Compute the end-effector pose in terms of the symbolic joint angles
Q = [q1v, q2v, q3v-q2v, -q3v, q4v];
TeqDH = Robeq.fkine(Q);
Xeq = TeqDH.t(1);
Yeq = TeqDH.t(2);
Zeq = TeqDH.t(3);
disp("The final transformation from base to EE");
display(simplify(TeqDH));

%% Jacobian
J = jacobian([Xeq;Yeq;Zeq],[q1v,q2v,q3v]);
J = subs(J, [L1v, L2v, L3v, L4v], [L1, L2, L3, L4]);
display(simplify(J));
detJ = det(J);
solutions = solve(detJ,[q1v,q2v,q3v]);
disp(simplify(detJ));
disp(solutions);

%% A Funciton to calculate the svd of iterations of the Jacobian Matrix
% across the range of the joints:
SampleRate = 50; 
n1 = ((qmax(1)-qmin(1))/SampleRate)+1; % 180/SampelRate = 1800 points
n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
n3 = ((qmax(3)-qmin(3))/SampleRate)+1;

q1 = linspace(theta1_min,theta1_max,n1);
q2 = linspace(theta2_min,theta2_max,n2);
q3 = linspace(theta3_min,theta2_max,n3);

% Jn = subs(J, [q1v, q2v, q3v], [q1(i), q2(j), q3(k)]); S = svd(Jn);

for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3) 
            Jn = subs(J, [q1v, q2v, q3v], [q1(i), q2(j), q3(k)]); 
            [U, S, V] = svd(Jn);
            if diag(S) == 0
                singular_values_cell{i, j, k} = 0; %#ok<SAGROW> 
            else
                singular_values_cell{i, j, k} = 1; %#ok<SAGROW> 
            end
%             % Check for singular values close to zero tolerance = 1e-6; %
%             Adjust as needed near_singularities = find(singular_values <
%             tolerance);
        end
    end
end

%% 2. Inverse Kinematics Function:
% Call KR700Inverse to calculate first 3 joints' variables
% If currentPose is provided, it will be used as an initial guess
% Otherwise, a default initial guess [0,42.5,52.5,0] is used
% 'interior-point' handles large, sparse problems, as well as small dense problems.
% The algorithm satisfies bounds at all iterations, and can recover from NaN or Inf results. 
% How to Use: [endEffector, currentPose] = End_effector_position;
function [Q, numIterations, timetaken] = KR700Inverse(endEffector,currentPose,showOutput)
    % Robot Link Lengths
  
    L1 = 742.5; % To match the KR700 Studio L1 = 0.
    L2 = 945; % length of link 2
    L3 = 1025; % length of link 3
    % L4 = 22.0;  % length of link 4. L4 = 61 To match with RoboDk Model.
    L4 = 485;
    L5 = 251.5-2.5;

    % Define the objective function (error to minimize)

    % Set default value for showOutput argument
    if nargin < 3
        showOutput = true; % Display text by default
    end

    objective = @(q) norm([(cos(q(1))*(L4 + L2*sin(q(2)) + L3*cos(q(3)))) - endEffector(1);
                           (sin(q(1))*(L4 + L2*sin(q(2)) + L3*cos(q(3)))) - endEffector(2);
                           (L1 - L5 + L2*cos(q(2)) - L3*sin(q(3))) - endEffector(3)]);
    % Initial guess:     initial_guess = [0, 42.5, 52.5, 0];

    % Set initial_guess to currentPose if it's supplied, otherwise use [0, 0, 0, 0]
    if nargin < 2
        initial_guess = [0, deg2rad(42.5), deg2rad(52.5), 0];
    else
        initial_guess = S2R(currentPose);
    end

    % No constraints for now
    qmin =  [-165 , -40, -20 , -300];
    qmax =  [+165 , +85, +120, +300];
    % Define lower and upper bounds (optional, if needed)
    lb = [deg2rad(qmin(1)), deg2rad(qmin(2)), deg2rad(qmin(3)), deg2rad(qmin(4))];
    ub = [deg2rad(qmax(1)), deg2rad(qmax(2)), deg2rad(qmax(3)), deg2rad(qmax(4))];
    % Call fmincon
    % options = optimoptions('fmincon','Display', 'iter','MaxIterations', 200,'TolFun', 1e-8 ,'TolX', 1e-8);
   
    if showOutput
        tic;
        options = optimoptions('fmincon','Algorithm','sqp', 'Display', 'iter','MaxIterations', 200,'TolFun', 1e-8 ,'TolX', 1e-8);
    else
        options = optimoptions('fmincon','Algorithm','sqp', 'Display', 'off','MaxIterations', 200,'TolFun', 1e-8 ,'TolX', 1e-8);
    end

%     if showOutput
%         tic;
%         options = optimoptions('fmincon', 'Display', 'iter','MaxIterations', 200,'TolFun', 1e-8 ,'TolX', 1e-8);
%     else
%         options = optimoptions('fmincon', 'Display', 'off','MaxIterations', 200,'TolFun', 1e-8 ,'TolX', 1e-8);
%     end

    [q_optimal, ~, ~, output] = fmincon(objective, initial_guess, [], [], [], [], lb, ub, [], options);


    if showOutput
        timetaken = toc;
    end

    % Access the number of iterations from the output structure
    numIterations = output.iterations;
    % Calculate Q values using DH parameters
    Q = [q_optimal(1), q_optimal(2), q_optimal(3) - q_optimal(2), -q_optimal(3), q_optimal(4)]; 

end


%% 3. Function to update the plot when the slider value changes:
function sliderCallback(source, ~ , A)
    [init_azimuth, init_elevation] = view;
    Shrink_Factor = source.Value;     % Gets the current value of the slider
    
    [k_A, ~] = boundary(A, Shrink_Factor); % Re-calculate and Re-plot
    [~, centroid1] = kmeans(A , 1);
    plot3(centroid1(1), centroid1(2), centroid1(3), "+", "MarkerSize", 15, "Color", "black", "LineWidth", 2);
    trisurf(k_A, A(:,1), A(:,2), A(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.5);
    view(init_azimuth, init_elevation);
    axis equal;
    hold off;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title("KR700 Workspace with AlphaShape: Sf = " + Shrink_Factor);

    drawnow; % Re-plot
end


%% 5. Function to convert real manipulator variables into simulation:
function Qout = R2S(Qin)
    Qout = [Qin(1),Qin(2),Qin(3)-Qin(2),-Qin(3),Qin(4)];
end

%% 6. Function to convert simulation variables into real manipulator:
function qout = S2R(qin)
    qout = [qin(1),qin(2),-qin(4),qin(5)];
end

%% 7. Function to calculate and plot the path for the robot

function [Qsol, Qreal] = PathPlan(theRobot,Path,ShowOutput)
    % Plotting the entire path
    figure;
    gif('Path.gif','DelayTime',1/10,'LoopCount',1000);

    theRobot.plot([0,0,0,0,0]);
    hold on;
    plot3(Path.X, Path.Y, Path.Z, 'r.-', 'LineWidth', 2);
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Path through Waypoints');

    n = length(Path.X);
    Qsol = zeros(n, 5); % Initializing Qsol for all waypoints
    Qreal = zeros(n,4);

    for i = 1:n
        TP = [Path.X(i), Path.Y(i), Path.Z(i)];
        IntGuess = theRobot.getpos; % Starting solution from current point
        
        % Call the inverse kinematics function (KR700Inverse) to compute Qsol
        [Qsol(i, :)] = KR700Inverse(TP, IntGuess, ShowOutput);
    
        if ShowOutput        
            % Display the joint angles in degrees
            disp('Q values: '); 
            disp(rad2deg(Qsol(i, :)));
        end
    end
    for i = 1:n
        % Plot the robot for each waypoint
        theRobot.plot(Qsol(i, :), 'view', [40 20], 'wrist', 'jaxes', 'arrow', 'jointcolor', [0.3, 0.3, 0.3], 'linkcolor', [1, 1, 1], 'tile1color', [0.9, 0.9, 0.9], 'tile2color', [0.9, 0.9, 0.9]);
        gif;
    end

    % Plotting the path again for clarity
    plot3(Path.X, Path.Y, Path.Z, 'r.-', 'LineWidth', 2);
    hold off;
end
