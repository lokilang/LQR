clear all, close all, clc;

%% Define the system

m = 0.05;
l = 0.48;
R = 0.57;
J = 0.03264;
b = 0.00351;
g = 9.8;

A = [0	1	0	0;
     (m*g*R*R + J*g)/J*l	0	0	(b*R)/(J*l);
     0	0	0	1;
     -(m*g*R)/J	0	0	-b/J];

B = [0;
     -R/J*l;
     0;
     1/J];
    
C = [1 0 0 0;
     0 0 1 0];
 
D = [0;0];

open = eig(A);
% the eigenvalues are
% 0
% -2.7316
% 2.5764
% 0.0476
% it is not stable

%% LQR design

Q = C'*C;
%Q = 100*eye(4);

R = 1;

[K,S,e] = lqr(A,B,Q,R);
% K = [-15.4872   -6.9955   -1.0000   -1.5095]

%% State feedback controller

sys = ss(A-B*K, eye(size(A)), eye(size(A)), eye(size(A)))

closed = eig(sys.a);
% the eigenvalues are
% -4.1307 + 3.7968i
% -4.1307 - 3.7968i
% -2.1193 + 0.2947i
% -2.1193 - 0.2947i
% it is stable

x0 = [0.5; 0; 0.4; 0];
% initial state

t = 0:0.01:5;
% the time duration

x = initial(sys,x0,t);
% response of the state-space
% with initial condition x0
% and time duration t
xd = [0 0 0 0];
x1 = [1 0 0 0]*x';%(xd'-x');
x2 = [0 1 0 0]*x';%(xd'-x');
x3 = [0 0 1 0]*x';%(xd'-x');
x4 = [0 0 0 1]*x';%(xd'-x');
% the state variables

d1 = zeros(size(t));
d2 = zeros(size(t));
d3 = zeros(size(t));
d4 = zeros(size(t));

%% Plot the state variables
figure(1);
subplot(4,1,1);
hold on;
plot(t,x1), plot(t,d1), grid
title('Response to initial condition')
ylabel('x1')
ylim([-0.6 0.6])

subplot(4,1,2);
hold on;
plot(t,x2), plot(t,d2), grid
ylabel('x2')
ylim([-4 2])

subplot(4,1,3);
hold on;
plot(t,x3), plot(t,d3), grid
ylabel('x3')
ylim([-1 5])

subplot(4,1,4);
hold on;
plot(t,x4), plot(t,d4), grid
ylabel('x4')
ylim([-7 16]);
xlabel('t (seconds)')

%%

figure(2);
hold on;
plot(t,x1), plot(t,d1), grid
title('Response to initial condition')
xlabel('Time (seconds)')
ylabel('Pendulum angle (radians)')
legend('$\bf{\theta}$', 'Reference', 'Interpreter', 'latex', 'Location', 'best')
ylim([-0.6 0.6]);

%%

figure(3);
hold on;
plot(t,x2), plot(t,d2), grid
title('Response to initial condition')
xlabel('Time (seconds)')
ylabel('Pendulum angle rate (radians/second)')
legend('$\bf{\dot{\theta}}$', 'Reference', 'Interpreter', 'latex', 'Location', 'best')
ylim([-4 2]);

%%

figure(4);
hold on;
plot(t,x3), plot(t,d3), grid
title('Response to initial condition')
xlabel('Time (seconds)')
ylabel('Arm angle (radians)')
legend('$\bf{\alpha}$', 'Reference', 'Interpreter', 'latex', 'Location', 'best')
ylim([-1 5])

%%

figure(5);
hold on;
plot(t,x4), plot(t,d4), grid
title('Response to initial condition')
xlabel('Time (seconds)')
ylabel('Arm angle rate (radians/second)')
legend('$\bf{\dot{\alpha}}$', 'Reference', 'Interpreter', 'latex', 'Location', 'best')
ylim([-7 16]);