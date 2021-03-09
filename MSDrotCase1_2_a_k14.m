% This function evaluates the state-space equations at each time called by
% the ode45 function (or other integrator)

function xdot=MSDrotCase1_2_a_k14(t,x)
%% Section 1 - Case 1&2

k = 14; % angular stiffness of the spring [N.m] 
c = 0; % angular damping coefficient of the damper [N.m.s] 
%% Section 2 - Case 3&4
% k = 5; % angular stiffness of the spring [N.m] % 
% c = 0.2; % angular damping coefficient of the damper [N.m.s] 

%% Section 3
% Specify system parameters

m= 1; % mass [kg] 
L = 1; % length of rod [m]
J=m*L^(2); % rotational moment of inertia
R=c;        % R=c in rotational motion
I=J;        % I=J in rotational motion
C=1/k;    % C=1/k in rotational motion [1/N.m]
g=9.81; % acceleration due to gravity [m/s^2]


%% Nonlinear case
xdot1 = -(R/I)*x(1)-(1/C)*x(2)+m*g*L*sin(x(2));
xdot2 = (1/I)*x(1);

xdot=[xdot1;xdot2];



