% clear;
% clc;
% close all;

%%

% load('ensaio.mat')
% ensaio = ensaio;
% tsim = ensaio.time(end);
% 
% Ts = mean(diff(ensaio.time));

model.b = 0.25e-1;
model.M = 0.90;
model.L = 0.45;
model.g = 9.81;
model.x = 0.012;
model.J = (model.M*(2*model.L)^2)/12;
model.gain = 0.0016*(model.J)*1.2*100;

%% Filtered PID with pole placement

tf_model = tf(model.gain,[model.J model.b model.M*model.g*model.x]);

qsi = 1;
tr = stepinfo(tf_model).SettlingTime/3;
wn = 4.74/tr;

tf1 = c2d(tf_model,Ts,'zoh');

[num,den] = tfdata(tf1,'v');
[~,den_mf] = tfdata(c2d(tf(wn^2,[1 2*qsi*wn wn^2]),Ts,'zoh'),'v');

b1 = num(2);
b2 = num(3);
a1 = den(2);
a2 = den(3);
p1 = den_mf(2);
p2 = den_mf(3);

r0_meq = (1+p1+p2)/(b1+b2);
r1_meq = a1*r0_meq;
r2_meq = a2*r0_meq;
s1_meq = r0_meq*b2-p2;
% 
% controller = tf([r0_meq r1_meq r2_meq],conv([1 -1],[1 s1_meq]),Ts);
% 
% figure
% subplot(211)
% step(feedback(controller*tf1,1))
% subplot(212)
% step(feedback(controller,tf1))

% sim("ModelPendulumTest")

%% State-space model

ssmodel.A = [0 1 0; -(model.M*model.g*model.x)/model.J -model.b/model.J 0; -1 0 0];
ssmodel.B = [0 model.gain/model.J 0; 0 -model.gain/model.J 0]';
ssmodel.C = [1 0 0; 0 1 0];
ssmodel.D = 0;

ss_model = ss(ssmodel.A,ssmodel.B,ssmodel.C,ssmodel.D);

ssmodel.R = 1e-4*eye(2);
ssmodel.Q = diag([1e-2,5e-2,10e-2]);

[K_controller,~,~] = lqr(ssmodel.A,ssmodel.B,ssmodel.Q,ssmodel.R)

sim("ModelPendulumTest")