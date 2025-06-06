clear;
close all;
clc;

%% Init 

%%%% Pin numbers
pot_pin = 0; %A0
motor1 = 9; %D9 - Motor Esquerda
motor2 = 10; %D10 - Motor Direita

%%%% Sampling angle measure
Ts_a = 5e-2;

%%%% Sampling control task
Ts = 5e-2;

%%%% Constants
pot_offset = single(512);
pot_gain = single(180/516);

%%%% Adjust output signal

% Zero velocity - 1ms
% Max velocity - 2ms
% Assumed max velocity - 1.5ms;

pwm_freq = single(490.2); % There is a bug in the connected IO mode that forces the output PWM to 490.2 Hz
min_dc = (1e-3/(1/pwm_freq))*255;
max_dc = (1.5e-3/(1/pwm_freq))*255;%(1.1e-3/(1/pwm_freq))*255;
pwm_gain = (max_dc-min_dc);

% Initialization variables
min_dc_init = (1e-3/(1/pwm_freq))*255;
max_dc_init = (2e-3/(1/pwm_freq))*255;

%% Steps Generation
% 1st method
% Independent actuators
%%% Input vector
% inputM1_time = single([0 5 5+Ts 35 35+Ts 65 65+Ts 95 95+Ts 125 125+Ts 155 155+Ts 185 185+Ts 215 215+Ts 245 245+Ts 275 275+Ts 305 305+Ts 335 335+Ts 365 365+Ts 395 395+Ts 425 425+Ts 480 480+Ts].')+15;
% inputM1 = single([0 0 20 20 20 20 30 30 35 35 35 35 40 40 45 45 45 45 50 50 55 55 55 55 60 60 65 65 65 65 70 70 0].');
% 
% inputM2_time = [0; inputM1_time(2:end-2)+15; inputM1_time(end-1:end)];
% inputM2 = single([0 0 20 20 30 30 30 30 35 35 40 40 40 40 45 45 50 50 50 50 55 55 60 60 60 60 65 65 70 70 70 70 0].');
% plot(inputM1_time,inputM1, inputM2_time,inputM2)

% 2nd method
% Equivalent actuator around 50% of motors
%%% Input Vector
% inputMeq_time = single([0 20 20+Ts 30 30+Ts 45 45+Ts 50 50+Ts 60 60+Ts 75 75+Ts 80].');
% inputMeq = single([0 0 5 5 -7 -7 2 2 8 8 -4 -4 0 0].');
% plot(inputMeq_time,inputMeq)

%3rd method - PRBS
%1/(Tb(2^N - 1)) < f_pbrs < 0,44/Tb

%% PRBS Generation
Tb = 3;
n = 5;
fmax = 0.44/Tb;
fmin = 1/((2^n - 1)*Tb);
fprintf('Frequência minima - máxima: %fHz - %fHz\nTempo do ensaio mínimo: %ds', fmin, fmax, (2^n - 1)*Tb);

prbs = [0 0];
base = [0 1 1 1 1];
t_prbs = [0 20];
for i=1:(2^n)
    prbs = [prbs base(end) base(end)];
    t_prbs = [t_prbs t_prbs(end)+Ts t_prbs(end)+Tb];
    base = [xor(base(3),base(5)) base(1:end-1)];
end

prbs(prbs>0) = 0.05;
prbs(prbs<=0) = -0.05;
prbs(1:2) = 0;
%%Duplica para ensaios consecutivos;
plot(t_prbs,prbs)

%% Estimated model parameters

model.b = 0.25e-1;
model.M = 0.90;
model.L = 0.45;
model.g = 9.81;
model.x = 0.012;
model.J = (model.M*(2*model.L)^2)/12;
model.gain = 0.0016*(model.J)*1.2*100*100;

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

controller = tf([r0_meq r1_meq r2_meq],conv([1 -1],[1 s1_meq]),Ts);

figure
subplot(211)
step(feedback(controller*tf1,1))
subplot(212)
step(feedback(controller,tf1))

%% RST Controller with pole placement

% Regulation

qsi = 0.7;
tr = stepinfo(tf_model).SettlingTime/10;
wn = 2.9/tr;
[~,den_mf] = tfdata(c2d(tf(wn^2,[1 2*qsi*wn wn^2]),Ts,'zoh'),'v');

Pz_RST = den;
    
Sp_RST = [1 -1];
Rp_RST = 1;
    
Cz_RST = conv(den,Sp_RST)';      
Ez_RST = conv(num,Rp_RST)';    % Considérant tous les zéros comme instables
nc_RST = length(Cz_RST)-1;          %
ne_d_RST = length(Ez_RST)-1;        %
ns_RST = ne_d_RST - 1;              % Calcul du degré des polynômes
nr_RST = nc_RST - 1;                %
np_RST = nc_RST + ne_d_RST - 1;     %


M_RST = [];
for i = 1:ns_RST+1
    M_RST = [M_RST [zeros(i-1,1);Cz_RST;zeros(ns_RST-i+1,1)]];
end
for i = 1:nr_RST+1
    M_RST = [M_RST [zeros(i-1,1);Ez_RST(1:end);zeros(nr_RST-i+1,1)]];
end

M_RST;

P_RST = conv(den_mf, Pz_RST);
alpha = 0.2;
Pa_RST = [1 -alpha];
for i = 1:(np_RST-length(P_RST)+1)
    P_RST = conv(Pa_RST, P_RST);
end

P_RST';

X_RST = M_RST\P_RST';
D_RST = X_RST(1:ns_RST+1)';
F_RST = X_RST(ns_RST+2:end)';
S = round(conv(Sp_RST,D_RST),3)
R = round(conv(F_RST,Rp_RST),3)
T = P_RST/sum(num)


qsi = 1;
tr = stepinfo(tf_model).SettlingTime/3;
wn = 4.74/tr;
[num_mf_track,den_mf_track] = tfdata(c2d(tf(wn^2,[1 2*qsi*wn wn^2]),Ts,'zoh'),'v');

%% State-space model (extended model)

ssmodel.A = [0 1 0; -(model.M*model.g*model.x)/model.J -model.b/model.J 0; -1 0 0];
ssmodel.B = [0 model.gain/model.J 0; 0 -model.gain/model.J 0]';
ssmodel.C = [1 0 0];
ssmodel.D = 0;

%ss_model = ss(ssmodel.A,ssmodel.B,ssmodel.C,ssmodel.D);

ssmodel.R = 0.05e-4*eye(2);
ssmodel.Q = diag([0.1e-2,0.1e-2,1e-2]);

[K_controller,~,poles] = lqr(ssmodel.A,ssmodel.B,ssmodel.Q,ssmodel.R)

%% State-space model with CNLL (extended model)

ssmodel2.A = [0 1 0; 0 0 0; -1 0 0];
ssmodel2.B = [0 1 0]';
ssmodel2.C = [1 0 0; 0 1 0];
ssmodel2.D = 0;

%ss_model2 = ss(ssmodel2.A,ssmodel2.B,ssmodel2.C,ssmodel2.D);

ssmodel2.R = 1e-4*eye(1);
ssmodel2.Q = diag([0.1e-2,0.1e-2,1e-2]);

[K_controller2,~,poles] = lqr(ssmodel2.A,ssmodel2.B,ssmodel2.Q,ssmodel2.R)

%sim("ModelPendulumTest.slx")

%% Luenberger Observer

ssmodel_A = [0 1; -(model.M*model.g*model.x)/model.J -model.b/model.J];
ssmodel_B = [0 model.gain/model.J]';
ssmodel_C = [1 0];
ssmodel_D = 0;

p_observer = [-8 -9];
L_gain = acker(ssmodel_A',ssmodel_C',p_observer)'

%% MPC

a = den;
b = num;
C = conv([1 0],conv([1 -0],[1 -0]));
d = 0;

a_t = conv([1 -1],a);
mpc_N = 100;
mpc_Rr = 100*eye(mpc_N);
mpc_Ru = 1*eye(mpc_N);
mpc_T = tril(ones(mpc_N));
mpc_Q = 1000*eye(mpc_N);

%%%% State space representation on canonical format

mpc_A = [-a_t(2:end)' [diag(ones(1,length(a_t)-2)); zeros(1,length(a_t)-2)]];
mpc_B = b';
mpc_D = C(2:end)'-a_t(2:end)';
mpc_H = [1 zeros(1,length(mpc_A)-1)];

for i = 1:mpc_N
    mpc_F(i,:) = mpc_H*mpc_A^i;
end

mpc_E = [mpc_H*mpc_D; mpc_F(1:end-1,:)*mpc_D];

mpc_Ga = [mpc_H*mpc_B; mpc_F(1:end-1,:)*mpc_B];
mpc_G = zeros(mpc_N);
for i = 1:mpc_N
    mpc_G(i:size(mpc_G,1)+1:mpc_N^2) = mpc_Ga(i);
end
mpc_G = tril(mpc_G);

%mpc_G = tril(toeplitz(mpc_Ga(1:mpc_N), zeros(1,mpc_N)));

aux = (mpc_G'*mpc_Q*mpc_G+mpc_Rr+mpc_T'*mpc_Ru*mpc_T)\mpc_G'*mpc_Q;
mpc_K = aux(1,:);
mpc_Kr = sum(mpc_K)
mpc_KF = mpc_K*mpc_F
mpc_KE = mpc_K*mpc_E

aux2 = (mpc_G'*mpc_Q*mpc_G+mpc_Rr+mpc_T'*mpc_Ru*mpc_T)\mpc_T'*mpc_Ru*mpc_T*ones(mpc_N,1);
mpc_Ku = aux2(1)
sim("ModelPendulumTest.slx")
