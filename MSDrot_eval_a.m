% Example ODE45 evaluation of a mass-spring-damper system
%
% m-files needed: MSDrot.m
%

% Last modified: 05 October 2020

close all;
clear;
%% Specify time range of interest and intial conditions
tspan=[0: 0.01: 10];   % Time range, [s]
m=1;
L=1;
J=m*L^2;
C1theta0=0.5; C1thetadot0=0; % Case 1 Initial displacement and angular velocity
% C2theta0=0; C2thetadot0=1; % Case 2 Initial displacement and angular velocity
% C3theta0=0.1; C3thetadot0=0; % Case 3 Initial displacement and angular velocity
% C4theta0=0.1; C4thetadot0=3.5; % Case 4 Initial displacement and angular velocity

C1x0=[J*C1thetadot0;C1theta0];       % C1 Initial Conditions, p3;q4 so Jthetadot0;theta0. % The J is  to convert from angular  velocity to angular momentum
% C2x0=[J*C2thetadot0;C2theta0];       % C1 Initial Conditions, p3;q4 so Jthetadot0;theta0. % The J is  to convert from angular  velocity to angular momentum
% C3x0=[J*C3thetadot0;C3theta0];       % C1 Initial Conditions, p3;q4 so Jthetadot0;theta0. % The J is  to convert from angular  velocity to angular momentum
% C4x0=[J*C4thetadot0;C4theta0];       % C1 Initial Conditions, p3;q4 so Jthetadot0;theta0. % The J is  to convert from angular  velocity to angular momentum
% 
k=10:2:20

figure
hold
% 
% %C1x0(3)=k(i)
% [C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k10,tspan,C1x0);
% txt = 'k = 10';
% plot(C10_t,C10_x(:,1)/J, 'Displayname', txt )
% 
% 
% [C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k12,tspan,C1x0);
% txt = 'k = 12';
% plot(C10_t,C10_x(:,1)/J, 'Displayname', txt)


[C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k14,tspan,C1x0);
txt = 'k = 14';
plot(C10_t,C10_x(:,1)/J, 'Displayname', txt)

% 
% [C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k16,tspan,C1x0);
% txt = 'k = 16';
% plot(C10_t,C10_x(:,1)/J, 'Displayname', txt)
% 
% [C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k18,tspan,C1x0);
% txt = 'k = 18';
% plot(C10_t,C10_x(:,1)/J, 'Displayname', txt)
% 
% [C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k20,tspan,C1x0);
% txt = 'k = 20';
% plot(C10_t,C10_x(:,1)/J, 'Displayname', txt)
% 
% [C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k22,tspan,C1x0);
% txt = 'k = 22';
% plot(C10_t,C10_x(:,1)/J, 'Displayname', txt)
% 
% [C10_t,C10_x] = ode45(@MSDrotCase1_2_a_k24,tspan,C1x0);
% txt = 'k = 24';
% plot(C10_t,C10_x(:,1)/J, 'Displayname', txt)

title('Inverted Pendulum Veloctiy Vs Time for k=14')
xlabel('Time, [s]')
ylabel('Angular Velocity, [θ/s]')
%legend show



%% Integrate state-space equations
%[C1_t,C1_x] = ode45(@MSDrotCase1_2_a,tspan,C1x0,k);
% [C2_t,C2_x] = ode45(@MSDrotCase1_2_a,tspan,C2x0);
% [C3_t,C3_x] = ode45(@MSDrotCase3_4_a,tspan,C3x0);
% [C4_t,C4_x] = ode45(@MSDrotCase3_4_a,tspan,C4x0);
% 

%% Convert angualar momentum state variable to the angular velocity of the mass
%qdot=C1_x1(:,1)/J;

%% Plot results
% figure
% plot(C1_t,C1_x(:,2))
% title('Inverted Pendulum MKB Response - Case 1')
% xlabel('Time, [s]')
% ylabel('Angular Displacement, [θ]')
% legend('θ')
% 
% figure
% plot(C2_t,C2_x(:,2))
% title('Inverted Pendulum MKB Response - Case 2')
% xlabel('Time, [s]')
% ylabel('Angular Displacement, [θ]')
% legend('θ')
% 
% figure
% plot(C3_t,C3_x(:,2), C4_t,C4_x(:,2))
% title('Inverted Pendulum MKB Response - Cases 3 & 4')
% xlabel('Time, [s]')
% ylabel('Angular Displacement, [θ]')
% legend('θdot(0)=0','θdot(0)=3.5')

% figure
% plot(C3_t,C3_x(:,2))
% title('Inverted Pendulum MKB Response - Case 3')
% xlabel('Time, [s]')
% ylabel('Angualr Displacement, [θ]')
% legend('θ')
% 
% figure
% plot(C4_t,C4_x(:,2))
% title('Inverted Pendulum MKB Response - Case 4')
% xlabel('Time, [s]')
% ylabel('Angualr Displacement, [θ]')
% legend('θ')

% figure
% plot(t,x(:,2),t,qdot)
% title('Spring-Mass-Damper Response')
% xlabel('Time, [s]')
% ylabel('State Variable')
% legend('x','v')