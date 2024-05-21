clear all
close all

%% Open loop transfer function

J=0.01; % moment of inertia of the rotor 
b=0.1;  % damping ratio of the mechanical system
k=0.01; % electromotive force constant
R=1;  % Electric resistance
L=0.5; % Electric inducatance

num=k; % numerator
den=[ (J*L) ((J*R)+(L*b)) ((b*R) + k^2)]; % denominator 

sys_OL = tf([num], [den]); % open loop transfer function

%% PID controller and Closed loop transfer function 

kp= 100; % proportional gain
ki= 200; % integral time constant
kd= 10; % derivative time constant

sys_PID = tf([kd kp ki], [1 0]); % transfer function of the PID controller
sys_CL = feedback(series(sys_OL, sys_PID), 1); % closed loop transfer  function

% pole(sys_CL)
% isstable(sys_CL)

%% Open Loop Response analysis

step(sys_OL, 0:0.01:10)
title("open loop response")
xlabel("time(sec)")
ylabel("amplitude")
stepinfo(sys_OL)

%% Closed loop reponse analysis

step(sys_CL, 0:0.01:2)
hold on
xlabel('time(sec)'), ylabel('velocity(rad/sec)') 
title ("pid control") 
grid; 

%% System performance analysis

yout = step(sys_CL, 0:0.01:2);

% settling time
yss = mean(yout(end-100:end));

% steady state error
ess = 1 - yss

% step info
stepinfo(sys_CL)

%% Check if system is observable/controllable

[A, B, C, D] = tf2ss(num, den); % from tf to state space
Ob = obsv(A, C); % observability matrix
det_Ob = det(Ob) % determinant of the obs. matrix

Co = ctrb(A,B); % controllability matrix
det_Co = det(Co) % determinant of the control. matrix

%% Simulink output

% steady state
yss = mean(out.outputData(901:end, 2))

% rise time
index_t90 = find(out.outputData(:, 2) > yss*0.9, 1, 'first');
index_t10 = find(out.outputData(:, 2) > yss*0.1, 1, 'first');
tr = out.tout(index_t90)-out.tout(index_t10)

%settling time
index_hi = find(out.outputData(:, 2) >= yss * 1.02, 1, 'last');
index_lo = find(out.outputData(:,2) <= yss * 0.98, 1, 'last');
index_ts = max([index_hi index_lo]);
ts = out.tout(index_ts)

%percentage overshoot
[maxout_sim indtp_sim] = max(out.outputData);
os = 100*(maxout_sim-yss)/yss


