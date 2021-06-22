clear all, close all, clc;

eb = 0.01;
%kc = 68.823e-06;
ic = 0.346024896;
Lc = 0.01;
Rc = 2.5;
Rs = 1;
g = 9.81

A = [0 1 0;
     (2*g)/eb 0 -(2*g)/ic;
     0 0 -(Rc+Rs)/Lc];

B = [0; 0; 1/Lc];

C = [1 0 0];

D = 0;

t = 0:0.001:1;
states = {'x' 'x_dot' 'i'};
inputs = {'u'};
outputs = {'x'};

sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A)

co = ctrb(sys);
controllability = rank(co)

Q  = 1000*eye(3)
R = 1;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

Cn = [1 0 0];
sys_ss = ss(A,B,Cn,0);
Nbar = rscale(sys_ss,K)

r = eb*ones(size(t));

ob = obsv(sys_ss);
observability = rank(ob)

poles = eig(Ac)

P = [-110 -111 -112];
L = place(A',C',P)'

Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B*Nbar;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = 0;

states = {'x' 'x_dot' 'i' 'e1' 'e2' 'e3'};
inputs = {'r'};
outputs = {'x'};

sys_est_cl = ss(Ace,Bce,Cce,Dce);%,'statename',states,'inputname',inputs,'outputname',outputs);
closed = eig(sys_est_cl.a);

[y,t,x]=lsim(sys_est_cl,r,t);

figure(1);
hold on;
plot(t,y), plot(t,r), grid;
title('Step Response with Observer-Based State-Feedback Control');
legend('x1', 'Reference', 'Location', 'best');
ylabel('Position (meters)');
xlabel('Time (seconds)')
ylim([0 0.011])


figure(2);
subplot(311)
plot(t,x(:,1)), grid;
title('States response to initial conditions');
ylabel('x1');
ylim([0 0.011]);

subplot(312)
plot(t,x(:,2)), grid;
ylabel('x2');

subplot(313)
plot(t,x(:,3)), grid;
ylabel('x3');
xlabel('Time (seconds)');
