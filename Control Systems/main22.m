close all
clear variables

l1 = 1;
l2 = 1;
m1 = 1;
m2 = 1;
g = 9.81;
tau1 = 0;
tau2 = 0;

%Kp = diag([1 1]);
%Kp = diag([0.2 1]);
Kp = diag([1 0.2]);
Kv = diag([2 2]);
q_d = [0; pi/2];
x_init = [-pi/2;0;0;0];
o0 = [0 0]'; 
C0 = eye(2);
o_d = C0*[1 1]'; 
od_dot = [0 0]';

sim('ClosedLoop');

%% Plot
figure
hold on;
subplot(4,1,1); 
plot(theta1); 
ylabel('theta1');
subplot(4,1,2); 
plot(theta2); 
ylabel('theta1');
subplot(4,1,3); 
plot(thetaDot1); 
ylabel('thetaDot1');
subplot(4,1,4); 
plot(thetaDot2); 
ylabel('thetaDot2');

figure
hold on;
plot(energy); plot(T_Kinetic, 'black'); plot(V_Potential, 'r');

figure
hold on;
plot(od_dot);