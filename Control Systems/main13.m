close all
clear variables


l1 = 1;
l2 = 1;
m1 = 1;
m2 = 1;
g = 9.81;
tau1 = -0.5;
tau2 = -0.5;
x_init = [0;0;0;0];

sim('openLoopThetaDot');

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
plot(T_Kinetic, 'r'); plot(V_Potential, 'black');