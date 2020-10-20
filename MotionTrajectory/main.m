close all
clear all
clc

%Below prompts a user for an end effector position/ orientation with
%regards to the approach and sliding vector and o_n.

fprintf("Program to generate a trajectory path to desired end position and orientation for Puma560\n");
prompt_endPosition = "Give the end effector position in vector form, for example [317;506;673]: ";
prompt_kd = "Give the end effector approach vector in vector form, for example [0.769; 0.401; 0.498]: ";
prompt_jd = "Give the end effector sliding vector in vector form, for example [-0.389; -0.325; 0.862]: ";
o_example = input(prompt_endPosition);
k_example = input(prompt_kd);
j_example = input(prompt_jd);

%Practice
% random = RandOrthMat(3);
% o_example = [371.5; 149.1; -100.1];
% j_example = [0;-1;0]; %random(:,2);
% k_example = [0; 0;1]; %random(:,3);

%Pass in the desired new position into genertion function
Puma560TrajectoryGeneration(o_example, k_example, j_example);

function [] = Puma560TrajectoryGeneration(o_n, k_n, j_n)
    % displacement arrays 
    theta1D = [];
    theta2D = [];
    theta3D = [];
    theta4D = [];
    theta5D = [];
    theta6D = [];
    
     % velocity arrays 
    theta1V = [];
    theta2V = [];
    theta3V = [];
    theta4V = [];
    theta5V = [];
    theta6V = [];
    
     % acceleration arrays 
    theta1A = [];
    theta2A = [];
    theta3A = [];
    theta4A = [];
    theta5A = [];
    theta6A = [];
    
     % acceleration arrays 2
    theta1A2 = [];
    theta2A2 = [];
    theta3A2 = [];
    theta4A2 = [];
    theta5A2 = [];
    theta6A2 = [];
    
    % time array
    time = [];
    
    % Bonus if have time
    % end effector positions/frames (Same as joint 6)
    endeffX = [];
    endeffY = [];
    endeffZ = [];
    endeffK = [];
    endeffJ = [];
    
    % joint 1 positions/frames
    j1X = [];
    j1Y = [];
    j1Z = [];
    j1K = [];
    j1J = [];
    
    % joint 2 positions/frames
    j2X = [];
    j2Y = [];
    j2Z = [];
    j2K = [];
    j2J = [];
    
    % joint 3 positions/frames
    j3X = [];
    j3Y = [];
    j3Z = [];
    j3K = [];
    j3J = [];
    
    % joint 4 positions/frames
    j4X = [];
    j4Y = [];
    j4Z = [];
    j4K = [];
    j4J = [];
    
    % joint 5 positions/frames
    j5X = [];
    j5Y = [];
    j5Z = [];
    j5K = [];
    j5J = [];
    
    % initialize our starting end effector position, orientation, and joint
    % parameters
    o_init = [471.5; 149.1; 433.1];
    i_init = [0; 0; 1];
    j_init = [0; -1; 0];
    k_init = [1; 0; 0];
    C_init = [i_init j_init k_init];
    q = [0; 0; deg2rad(90); 0; deg2rad(90); 0];
    
    % Calculate our C_n 
    i_n = cross(j_n, k_n);
    C_n = [i_n j_n k_n];
    
    %Let's calculate our change in translational position
    change_o = o_n - o_init;
    
    %Let's calculate our angular displacement
    R = C_n*inv(C_init);
    i = R(:,1);
    R(:,1) = i/norm(i);
    j = R(:,2);
    R(:,2) = j/norm(j);
    R(:,3) = cross((i/norm(i)),(j/norm(j)));
    axang = rotm2axang(R);
    s = axang(1:3);
    change_theta = (axang(4) * s);
    
    % calculate acceleration needed (as 6x1 vector)
    acceleration = calculateAcceleration(change_o, change_theta);
    
    %Setting frequency
    Fs = 200; dt = 1/Fs;
    
    %Set starting velocity
    v_n = [0;0;0;0;0;0];
    
    %Let's get the Jacobian of the q_init
    [J] = calculateJacobian(q);
    
    % keep track of change in q_dot_prev for secondary check on joint
    % accelerations
    q_dot_prev = [0; 0; 0; 0; 0; 0];
    
    %perform timesteps
    it = 1;
    for ts = 0:dt:1
        q_prev = q;
        
        % calculating our change in q based on equation 28 from 3.3.2
        q_dot = pinv(J) * v_n;
        
        % update q for this time step
        q = q_prev + q_dot;
        
        %update jacobian for next time loop.
        [J, o_6, C6, o_5, C5, o_4, C4, o_3, C3, o_2, C2, o_1, C1] = calculateJacobian(q);
        
        %bonus
        endeffX(it) = o_6(1);
        endeffY(it) = o_6(2);
        endeffZ(it) = o_6(3);
        endeffK(:,it) = C6(:,3);
        endeffJ(:,it) = C6(:,2);
        
        %joint 1
        j1X(it) = o_1(1);
        j1Y(it) = o_1(2);
        j1Z(it) = o_1(3);
        j1K(:,it) = C1(:,3);
        j1J(:,it) = C1(:,2);
        
        %joint 2
        j2X(it) = o_2(1);
        j2Y(it) = o_2(2);
        j2Z(it) = o_2(3);
        j2K(:,it) = C2(:,3);
        j2J(:,it) = C2(:,2);
        
        %joint 3
        j3X(it) = o_3(1);
        j3Y(it) = o_3(2);
        j3Z(it) = o_3(3);
        j3K(:,it) = C3(:,3);
        j3J(:,it) = C3(:,2);
        
        %joint 4
        j4X(it) = o_4(1);
        j4Y(it) = o_4(2);
        j4Z(it) = o_4(3);
        j4K(:,it) = C4(:,3);
        j4J(:,it) = C4(:,2);
        
        %joint 5
        j5X(it) = o_5(1);
        j5Y(it) = o_5(2);
        j5Z(it) = o_5(3);
        j5K(:,it) = C5(:,3);
        j5J(:,it) = C5(:,2);
        
        
        % use updated q to look at acceleration of joints via two methods
        % First is the Jacobian inverse equation 26 in 3.3.2
        % Second is common sense logic of the difference between q_dot(t) -
        % q_dot(t-1) / dt.
        J_dot = calculateJacobianInverse(q, q_dot);
        q_dot_dot2 = (q_dot - q_dot_prev) / dt;
        
        if(ts < 0.5)
            v_n = v_n + (acceleration*dt);
            q_dot_dot = pinv(J) * (acceleration - (J_dot*q_dot));
        end
        if(ts > 0.5)
            v_n = v_n - (acceleration*dt);
            q_dot_dot = pinv(J) * (-acceleration - (J_dot*q_dot));
        end
        
        % update q_dot_prev for next time loop
        q_dot_prev = q_dot;
        
        % add displacements, velocities, accelerations to our arrays
        theta1D(it) = q(1);
        theta2D(it) = q(2);
        theta3D(it) = q(3);
        theta4D(it) = q(4);
        theta5D(it) = q(5);
        theta6D(it) = q(6);
        
         % velocity arrays 
        theta1V(it) = q_dot(1);
        theta2V(it) = q_dot(2);
        theta3V(it) = q_dot(3);
        theta4V(it) = q_dot(4);
        theta5V(it) = q_dot(5);
        theta6V(it) = q_dot(6);
        
         % acceleration arrays 
        theta1A(it) = q_dot_dot(1);
        theta2A(it) = q_dot_dot(2);
        theta3A(it) = q_dot_dot(3);
        theta4A(it) = q_dot_dot(4);
        theta5A(it) = q_dot_dot(5);
        theta6A(it) = q_dot_dot(6);
        
         % acceleration arrays 2
        theta1A2(it) = q_dot_dot2(1);
        theta2A2(it) = q_dot_dot2(2);
        theta3A2(it) = q_dot_dot2(3);
        theta4A2(it) = q_dot_dot2(4);
        theta5A2(it) = q_dot_dot2(5);
        theta6A2(it) = q_dot_dot2(6);
        
        %time array
        time(it) = ts;
        
        it = it + 1;
    end
    
    % Plot figure for joint displacements (q), joint velocities(q_dot),
    % joint acceleration (q_dot_dot), and animation bonus.
    figure
    plot(time, theta1D);
    hold on
    plot(time, theta2D);
    hold on
    plot(time, theta3D);
    hold on
    plot(time, theta4D);
    hold on
    plot(time, theta5D);
    hold on
    plot(time, theta6D);
    hold off
    
    legend('joint 1 displacement', 'joint 2 displacement', 'joint 3 displacement', 'joint 4 displacement', 'joint 5 displacement', 'joint 6 displacement');
    
    figure
    plot(time, theta1V);
    hold on
    plot(time, theta2V);
    hold on
    plot(time, theta3V);
    hold on
    plot(time, theta4V);
    hold on
    plot(time, theta5V);
    hold on
    plot(time, theta6V);
    hold off
    
    legend('joint 1 velocity', 'joint 2 velocity', 'joint 3 velocity', 'joint 4 velocity', 'joint 5 velocity', 'joint 6 velocity');
    
    figure
    plot(time, theta1A);
    hold on
    plot(time, theta2A);
    hold on
    plot(time, theta3A);
    hold on
    plot(time, theta4A);
    hold on
    plot(time, theta5A);
    hold on
    plot(time, theta6A);
    hold off
    
    legend('joint 1 acceleration JI', 'joint 2 acceleration JI', 'joint 3 acceleration JI', 'joint 4 acceleration JI', 'joint 5 acceleration JI', 'joint 6 acceleration JI');
    
    figure
    plot(time, theta1A2);
    hold on
    plot(time, theta2A2);
    hold on
    plot(time, theta3A2);
    hold on
    plot(time, theta4A2);
    hold on
    plot(time, theta5A2);
    hold on
    plot(time, theta6A2);
    hold off
    
    legend('joint 1 acceleration delta', 'joint 2 acceleration delta', 'joint 3 acceleration delta', 'joint 4 acceleration delta', 'joint 5 acceleration delta', 'joint 6 acceleration delta');
    
    figure;
    hold on
    axesHandle = gca;
    xlim(axesHandle, [(-700) (700)]);
    ylim(axesHandle, [(-700) (700)]);
    zlim(axesHandle, [(-700) (700)]);
    [X,Y,Z] = sphere;
    
    x0 = 0;
    x1 = j1X(1);
    x2 = j2X(1);
    x3 = j3X(1);
    x4 = j4X(1);
    x5 = j5X(1);
    x6 = endeffX(1);
    
    y0 = 0;
    y1 = j1Y(1);
    y2 = j2Y(1);
    y3 = j3Y(1);
    y4 = j4Y(1);
    y5 = j5Y(1);
    y6 = endeffY(1);
    
    z0 = 0;
    z1 = j1Z(1);
    z2 = j2Z(1);
    z3 = j3Z(1);
    z4 = j4Z(1);
    z5 = j5Z(1);
    z6 = endeffZ(1);
    
    j1 = j1J(:,1);
    j2 = j1J(:,1);
    j3 = j2J(:,1);
    j4 = j3J(:,1);
    j5 = j4J(:,1);
    j6 = j5J(:,1);
    
    k1 = j1K(:,1);
    k2 = j1K(:,1);
    k3 = j2K(:,1);
    k4 = j3K(:,1);
    k5 = j4K(:,1);
    k6 = j5K(:,1);
    
    joint1J = line([x1 (x1+(100*j1(1)))],[y1 (y1+(100*j1(2)))], [z1 (z1+(100*j1(3)))]);
    set(joint1J,'Color','g');   %line 1 color
    joint1K = line([x1 (x1+(100*k1(1)))],[y1 (y1+(100*k1(2)))], [z1 (z1+(100*k1(3)))]);
    set(joint1K,'Color','r');   %line 1 color
    
    
    joint2J = line([x2 x2+(10*j2(1))],[y2 y2+(10*j2(1))], [z2 z2+(10*j2(1))]); %link segments
    set(joint2J,'Color','g');   %line 1 color
    joint2K = line([x2 x2+(10*k2(1))],[y2 y2+(10*k2(1))], [z2 z2+(10*k2(1))]); %link segments
    set(joint2K,'Color','r');   %line 1 color
    
    
    joint3J = line([x3 x3+(10*j3(1))],[y3 y3+(10*j3(1))], [z3 z3+(10*j3(1))]); %link segments
    set(joint3J,'Color','g');   %line 1 color
    joint3K = line([x3 x3+(10*k3(1))],[y3 y3+(10*k3(1))], [z3 z3+(10*k3(1))]); %link segments
    set(joint3K,'Color','r');   %line 1 color
    
    
    joint4J = line([x4 x4+(10*j4(1))],[y4 y4+(10*j4(1))], [z4 z4+(10*j4(1))]); %link segments
    set(joint4J,'Color','g');   %line 1 color
    joint4K = line([x4 x4+(10*k4(1))],[y4 y4+(10*k4(1))], [z4 z4+(10*k4(1))]); %link segments
    set(joint4K,'Color','r');   %line 1 color
    
    
    joint5J = line([x5 x5+(10*j5(1))],[y5 y5+(10*j5(1))], [z5 z5+(10*j5(1))]); %link segments
    set(joint5J,'Color','g');   %line 1 color
    joint5K = line([x5 x5+(10*k5(1))],[y5 y5+(10*k5(1))], [z5 z5+(10*k5(1))]); %link segments
    set(joint5K,'Color','r');   %line 1 color
    
    
    joint6J = line([x6 x6+(10*j6(1))],[y6 y6+(10*j6(1))], [z6 z6+(10*j6(1))]); %link segments
    set(joint6J,'Color','g');   %line 1 color
    joint6K = line([x6 x6+(10*k6(1))],[y6 y6+(10*k6(1))], [z6 z6+(10*k6(1))]); %link segments
    set(joint6K,'Color','r');   %line 1 color
    
    
    lineHandle1 = line([x0 x1],[y0 y1], [z0 z1]); %link segments
    lineHandle2 = line([x1 x2],[y1 y2], [z1 z2]); %link segments
    lineHandle3 = line([x2 x3],[y2 y3], [z2 z3]); %link segments
    lineHandle4 = line([x3 x4],[y3 y4], [z3 z4]); %link segments
    lineHandle5 = line([x4 x5],[y4 y5], [z4 z5]); %link segments
    lineHandle6 = line([x5 x6],[y5 y6], [z5 z6]); %link segments
    
    set(lineHandle1,'Color','b');   %line 1 color
    set(lineHandle2,'Color','b');   %line 1 color
    set(lineHandle3,'Color','b');   %line 1 color
    set(lineHandle4,'Color','b');   %line 1 color
    set(lineHandle5,'Color','b');   %line 1 color
    set(lineHandle6,'Color','b');   %line 1 color
    
    view(3);
    hold off
    for j=1:it-1
       
        drawnow; %Forces MATLAB to render the snake
        
        %%recalculate positions - 
        
        x0 = 0;
        x1 = j1X(j);
        x2 = j2X(j);
        x3 = j3X(j);
        x4 = j4X(j);
        x5 = j5X(j);
        x6 = endeffX(j);
        
        y0 = 0;
        y1 = j1Y(j);
        y2 = j2Y(j);
        y3 = j3Y(j);
        y4 = j4Y(j);
        y5 = j5Y(j);
        y6 = endeffY(j);
        
        z0 = 0;
        z1 = j1Z(j);
        z2 = j2Z(j);
        z3 = j3Z(j);
        z4 = j4Z(j);
        z5 = j5Z(j);
        z6 = endeffZ(j);
        
        j1 = j1J(:,j);
        j2 = j1J(:,j);
        j3 = j2J(:,j);
        j4 = j3J(:,j);
        j5 = j4J(:,j);
        j6 = j5J(:,j);
        
        k1 = j1K(:,j);
        k2 = j1K(:,j);
        k3 = j2K(:,j);
        k4 = j3K(:,j);
        k5 = j4K(:,j);
        k6 = j5K(:,j);
        
        %redraw 
        set(joint1J, 'Xdata',[x1 x1+(100*j1(1))],'Ydata', [y1 y1+(100*j1(2))], 'Zdata', [z1 z1+(100*j1(3))]);
        set(joint2J, 'Xdata',[x2 x2+(100*j2(1))],'Ydata', [y2 y2+(100*j2(2))], 'Zdata', [z2 z2+(100*j2(3))]);
        set(joint3J, 'Xdata',[x3 x3+(100*j3(1))],'Ydata', [y3 y3+(100*j3(2))], 'Zdata', [z3 z3+(100*j3(3))]);
        set(joint4J, 'Xdata',[x4 x4+(100*j4(1))],'Ydata', [y4 y4+(100*j4(2))], 'Zdata', [z4 z4+(100*j4(3))]);
        set(joint5J, 'Xdata',[x5 x5+(100*j5(1))],'Ydata', [y5 y5+(100*j5(2))], 'Zdata', [z5 z5+(100*j5(3))]);
        set(joint6J, 'Xdata',[x6 x6+(100*j6(1))],'Ydata', [y6 y6+(100*j6(2))], 'Zdata', [z6 z6+(100*j6(3))]);
        
        
        set(joint1K, 'Xdata',[x1 x1+(100*k1(1))],'Ydata', [y1 y1+(100*k1(2))], 'Zdata', [z1 z1+(100*k1(3))]);
        set(joint2K, 'Xdata',[x2 x2+(100*k2(1))],'Ydata', [y2 y2+(100*k2(2))], 'Zdata', [z2 z2+(100*k2(3))]);
        set(joint3K, 'Xdata',[x3 x3+(100*k3(1))],'Ydata', [y3 y3+(100*k3(2))], 'Zdata', [z3 z3+(100*k3(3))]);
        set(joint4K, 'Xdata',[x4 x4+(100*k4(1))],'Ydata', [y4 y4+(100*k4(2))], 'Zdata', [z4 z4+(100*k4(3))]);
        set(joint5K, 'Xdata',[x5 x5+(100*k5(1))],'Ydata', [y5 y5+(100*k5(2))], 'Zdata', [z5 z5+(100*k5(3))]);
        set(joint6K, 'Xdata',[x6 x6+(100*k6(1))],'Ydata', [y6 y6+(100*k6(2))], 'Zdata', [z6 z6+(100*k6(3))]);
        
        
        set(lineHandle1,'XData',[x0 x1],'YData',[y0 y1], 'ZData', [z0 z1]);
        set(lineHandle2,'XData',[x1 x2],'YData',[y1 y2], 'ZData', [z1 z2]);
        set(lineHandle3,'XData',[x2 x3],'YData',[y2 y3], 'ZData', [z2 z3]);
        set(lineHandle4,'XData',[x3 x4],'YData',[y3 y4], 'ZData', [z3 z4]);
        set(lineHandle5,'XData',[x4 x5],'YData',[y4 y5], 'ZData', [z4 z5]);
        set(lineHandle6,'XData',[x5 x6],'YData',[y5 y6], 'ZData', [z5 z6]);
        pause(0.01);
    end
    
end
