close all
clear all
clc

%Five distinc examples to verify that our code is working effectively.
%General Check
[e,s,a] = ForKinPuma560(20, 0, 20,0,30,0);
display(e);
display(s);
display(a);

InvKinPuma560(e, a, s);

%Negative angles check
[e,s,a] = ForKinPuma560(10, 15, -20,33,65,95);
display(e);
display(s);
display(a);

InvKinPuma560(e, a, s);

%Home position check
[e,s,a] = ForKinPuma560(0, 0, 0,0,0,0);
display(e);
display(s);
display(a);

InvKinPuma560(e, a, s);

%Invalid Joint Check
[e,s,a] = ForKinPuma560(180, 200, 200,200,200,300);
display(e);
display(s);
display(a);

InvKinPuma560(e, a, s);

%Singularity Check (Arm)
[e,s,a] = ForKinPuma560(0, 0, -2.67,0,0,0);
display(e);
display(s);
display(a);

InvKinPuma560(e, a, s);

fprintf("\n\n\n");

fprintf("Solution for the inverse kinematics puma560 example\n");
prompt_endPosition = "Give the end effector position in vector form, for example [317;506;673]: ";
prompt_kd = "Give the end effector approach vector in vector form, for example [0.769; 0.401; 0.498]: ";
prompt_jd = "Give the end effector sliding vector in vector form, for example [-0.389; -0.325; 0.862]: ";
o_example = input(prompt_endPosition);
k_example = input(prompt_kd);
j_example = input(prompt_jd);
InvKinPuma560(o_example, k_example, j_example);

function [endPosition, slidingVector, approachVector] = ForKinPuma560(theta1, theta2, theta3, theta4, theta5, theta6)
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);
    theta3 = deg2rad(theta3);
    theta4 = deg2rad(theta4);
    theta5 = deg2rad(theta5);
    theta6 = deg2rad(theta6);
    C0=eye(3);
    o0 = [0;0;0];
    T0 = [C0,o0];
    T0 = [T0;[0,0,0,1]];
    %Forward Kinematics using DH_Homog from Assignment 1
    [T1,C01] = DH_homog(theta1, 0, 0, -pi/2);
    C1 = C0*C01;
    endPosition = T0*T1;
    
    %Forward Kinematics
    [T2,C12] = DH_homog(theta2, 0, 431.8, pi);
    C2 = C1*C12;
    endPosition = endPosition*T2;
    
    %Forward Kinematics
    [T3,C23] = DH_homog(theta3 + pi/2, -149.09, 20.32, pi/2);
    C3 = C2*C23;
    endPosition = endPosition*T3;
    
    %Forward Kinematics
    [T4,C34] = DH_homog(theta4, 433.1, 0, pi/2);
    C4 = C3*C34;
    endPosition = endPosition*T4;
    
    %Forward Kinematics
    [T5,C45] = DH_homog(theta5, 0, 0, -pi/2);
    C5 = C4*C45;
    endPosition = endPosition*T5;
    
    %Forward Kinematics
    [T6,C56] = DH_homog(theta6, 60, 0, 0);
    C6 = C5*C56;
    endPosition = endPosition*T6;
    endPosition = endPosition(1:3,4);
    slidingVector=C6(:,2);
    approachVector=C6(:,3);
    
end

function [theta1, theta2, theta3, theta4, theta5, theta6] = InvKinPuma560(o_d, k_d, j_d) 
%Verify that kd and jd are orthogonal and have magnitude of 1
if(abs(norm(k_d) - 1) > 1000*eps("single") || abs(norm(j_d)-1) > 1000*eps("single"))
    fprintf("ERROR: the input k_d and j_d are not normalized.");
    return;
end
if(norm(cross(j_d,k_d)) > 10000000*eps("single"))
    fprintf("ERROR: the input k_d and j_d are not orthogonal.");
    return;
end
    
% Base Coordinate System
i_0 = [1;0;0];
j_0 = [0;1;0];
k_0 = [0;0;1];
o_0 = [0;0;0];
C_0 = [i_0, j_0, k_0];

ix=cross([i_0,i_0,i_0],eye(3)); % the skew symmetric operator on "i"
kx=cross([k_0,k_0,k_0],eye(3)); %the skew symmetric operator on "k"
jx=cross([j_0,j_0,j_0],eye(3)); %the skew symmetric operator on "j"
%Note we can calculate our wrist center by displacing our end position
%60mm in the -k_d direction.
k_6 = k_d;
o_4 = o_d - (60*k_6);
V = o_4;
i_d = cross(j_d, k_d);
C_6 = [i_d, j_d, k_d];

%First we find theta 3. This is because the length of our vector 
%V is only dependent on theta 2 and theta 3. Thus we can utilize
%kahan p4 to identify our theta 3.

%First we have to calculate our theta3 offset as we have a "kink"
%in the arm
a_home = 431.8;
b_home = sqrt(433.07^2 + 20.32^2);
c_home = sqrt((433.07+431.8)^2 + 20.30^2);
theta3_offset = -kahanP4(a_home, b_home, c_home);

% Next we calculate the new length of our c based on our end position 0_4
c = sqrt(o_4(1)^2 + o_4(2)^2 - 149.1^2 + o_4(3)^2); %Projected onto i1-j1 plane
theta3 = [+kahanP4(a_home, b_home, c) + theta3_offset; +kahanP4(a_home, b_home, c) + theta3_offset; -kahanP4(a_home, b_home, c) + theta3_offset; -kahanP4(a_home, b_home, c) + theta3_offset];


%Next we can find theta1 and theta2 via finding the vector from our origin to the wrist center
% if and only if theta3 is positioned as calculated above (while maintaining theta1 and theta2 =
% 0). Once we obtain this u (note there will be two for each of the "elbow up" and "elbow down" positions).
% We can use the formula (e^-theta1(k_0x)V = e^theta2(j_0x)U:
u_1 = [431.8 + (b_home*cos(theta3(1)));149.1;20.3+(b_home*sin(theta3(1)))];
u_2 = [431.8 + (b_home*cos(theta3(3)));149.1;20.3+(b_home*sin(theta3(3)))];

% Use Kahan P3 for this!
% two solutions returned for theta1 (left and right arm). Four
% possibilities for theta2.
[theta1, theta2] = kahanP3(k_0, j_0, V, u_1);
theta1 = -theta1;
[delta1, delta2] = kahanP3(k_0, j_0, V, u_2);
delta1 = -delta1;
theta1(3) = delta1(1);
theta1(4) = delta1(2);
theta2(3) = delta2(1);
theta2(4) = delta2(2);

%Now let's find our C3 frame using forward kinematics:
% Successively compute result for each of our 4 above possible solutions.
for i = 1:4
    if(isnan(theta1(i)) || isnan(theta2(i)) || isnan(theta3(i)))
        theta1(i+4) = NaN;
        theta2(i+4) = NaN;
        theta3(i+4) = NaN;
        theta4(i) = NaN;
        theta4(i+4) = NaN;
        theta5(i) = NaN;
        theta5(i+4) = NaN;
        theta6(i) = NaN; 
        theta6(i+4) = NaN;
        continue;
    end
    C0=eye(3);
    %Forward Kinematics using DH_Homog from Assignment 1
    [T1,C01] = DH_homog(theta1(i), 0, 0, -pi/2);
    C1 = C0*C01;
    
    %Forward Kinematics
    [T1,C12] = DH_homog(theta2(i), 0, 431.8, pi);
    C2 = C1*C12;
    
    %Forward Kinematics
    [T1,C23] = DH_homog(theta3(i) + pi/2, -149.09, 20.32, pi/2);
    C3 = C2*C23;
    
    %Utilize last assignment's function to gather theta4, theta5, and theta6
    [delta1, delta2, delta3] = InvKinSphWrist(C3, C_6);
    theta1(i+4) = theta1(i);
    theta2(i+4) = theta2(i);
    theta3(i+4) = theta3(i);
    theta4(i) = delta1(1);
    theta4(i+4) = delta1(2);
    theta5(i) = delta2(1);
    theta5(i+4) = delta2(2);
    theta6(i) = delta3(1);
    theta6(i+4) = delta3(2);
end

%Check solutions for validity and print out results
for i = 1:8
    fprintf("If %d solution not listed below, it was not valid or possibly close to a singularity\n", i);
    if(isnan(theta1(i)) || isnan(theta2(i)) || isnan(theta3(i)) || isnan(theta4(i)) || isnan(theta5(i)) ...
            || isnan(theta6(i)))
        continue;
    end
    %Check each joint motion range for validity
    if(rad2deg(theta1(i)) > 160 || rad2deg(theta1(i)) < -160)
        continue;
    end
    
    %Check each joint motion range for validity
    if(rad2deg(theta2(i)) > 45 || rad2deg(theta2(i)) < -225)
        continue;
    end
    
    %Check each joint motion range for validity
    if(rad2deg(theta3(i)) > 135 || rad2deg(theta3(i)) < -135)
        continue;
    end
    
    %Check each joint motion range for validity
    if(rad2deg(theta4(i)) > 170 || rad2deg(theta4(i)) < -110)
        continue;
    end
    
    %Check each joint motion range for validity
    if(rad2deg(theta5(i)) > 100 || rad2deg(theta5(i)) < -100)
        continue;
    end
    
    %Check each joint motion range for validity
    if(rad2deg(theta6(i)) > 266 || rad2deg(theta6(i)) < -266)
        continue;
    end
    fprintf("A possible solution %d is theta1 = %d, theta2 = %d, theta3 = %d, theta4 = %d, " + ...
        "theta5 = %d, theta6 = %d\n\n\n", i, rad2deg(theta1(i)), rad2deg(theta2(i)),rad2deg(theta3(i)), ...
        rad2deg(theta4(i)), rad2deg(theta5(i)), rad2deg(theta6(i)));
    if(rad2deg(theta6(i)) > 0 && -(360-rad2deg(theta6(i))) > -266)
        fprintf("A possible solution %d variation is theta1 = %d, theta2 = %d, theta3 = %d, theta4 = %d, " + ...
        "theta5 = %d, theta6 = %d\n\n\n", i, rad2deg(theta1(i)), rad2deg(theta2(i)),rad2deg(theta3(i)), ...
        rad2deg(theta4(i)), rad2deg(theta5(i)), -(360-rad2deg(theta6(i))) );
    end
    if(rad2deg(theta6(i)) < 0 && (360+rad2deg(theta6(i))) < 266)
        fprintf("A possible solution %d variation is theta1 = %d, theta2 = %d, theta3 = %d, theta4 = %d, " + ...
        "theta5 = %d, theta6 = %d\n\n\n", i, rad2deg(theta1(i)), rad2deg(theta2(i)),rad2deg(theta3(i)), ...
        rad2deg(theta4(i)), rad2deg(theta5(i)), (360+rad2deg(theta6(i))) );
    end
end
end

%Slight alterations made below to more effectively calculate last three 
%thetas. Specifically added a parameter C0 which can now be any orthogonal
%frame (in our case it is our calculated C3). Also utilized professor's
%secondary method to solving thetas found in answers as was more accurate
%for my results.
function [theta_1, theta_2, theta_3] = InvKinSphWrist(C0, C)
    theta_1 = [NaN; NaN];
    theta_2 = [NaN; NaN];
    theta_3 = [NaN; NaN];
    C3 = C;
    k_0 = C0(:,3);
    i_0 = C0(:,1);
    j_0 = C0(:,2);
    k_3 = C3(:,3);
    j_3 = C3(:,2);
    i_3 = C3(:,1);
    if(norm(cross(k_3,k_0)) > eps("single"))
        [theta_1, theta_3] = kahanP3(k_0, -k_3, j_0, j_3);
        k_1Plus = expm(theta_1(1)*skew(k_0))*j_0;
        k_1Minus = expm(theta_1(2)*skew(k_0))*j_0;
        theta_2 = [-kahanP2(k_1Plus,k_0,k_3); -kahanP2(k_1Minus,k_0,k_3)];
    end
end

%Made during Assignment 1: Used for forward kinematics
function [T, C] = DH_homog(theta, d, a, alpha)
    i=[1;0;0];
    k=[0;0;1];
    angle = [expm(theta*skew(k)) zeros(3,1); zeros(1,3) 1];
    offset = [eye(3) d*k; zeros(1,3) 1];
    length = [eye(3) a*i; zeros(1,3) 1];
    twist = [expm(alpha*skew(i)) zeros(3,1); zeros(1,3) 1];
    C = expm(theta*skew(k)) * expm(alpha*skew(i));
    T = angle*offset*length*twist;
end







