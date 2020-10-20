function [J, o_6, C6, o_5, C5, o_4, C4, o_3, C3, o_2, C2, o_1, C1] = calculateJacobian(q)
    o_0 = [0; 0; 0];
    C0=eye(3);
    o0 = [0;0;0];
    T0 = [C0,o0];
    T0 = [T0;[0,0,0,1]];
    
    %Forward Kinematics using DH_Homog from Assignment 1
    [T1,C01] = DH_homog(q(1), 0, 0, -pi/2);
    C1 = C0*C01;
    endPosition = T0*T1;
    o_1 = endPosition(1:3,4);
    
    %Forward Kinematics
    [T2,C12] = DH_homog(q(2), 0, 431.8, pi);
    C2 = C1*C12;
    endPosition = endPosition*T2;
    o_2 = endPosition(1:3,4);
    
    %Forward Kinematics
    [T3,C23] = DH_homog(q(3) + pi/2, -149.09, 20.32, pi/2);
    C3 = C2*C23;
    endPosition = endPosition*T3;
    o_3 = endPosition(1:3,4);
    
    %Forward Kinematics
    [T4,C34] = DH_homog(q(4), 433.1, 0, pi/2);
    C4 = C3*C34;
    endPosition = endPosition*T4;
    o_4 = endPosition(1:3,4);
    
    %Forward Kinematics
    [T5,C45] = DH_homog(q(5), 0, 0, -pi/2);
    C5 = C4*C45;
    endPosition = endPosition*T5;
    o_5 = endPosition(1:3,4);
    
    %Forward Kinematics
    [T6,C56] = DH_homog(q(6), 60, 0, 0);
    C6 = C5*C56;
    endPosition = endPosition*T6;
    o_6 = endPosition(1:3,4);
    display(o_6);
    display(C6);
    
    J0 = [ skew(C0(:,3))*(o_6 - o_0); C0(:,3)];
    J1 = [ skew(C1(:,3))*(o_6 - o_1); C1(:,3)];
    J2 = [ skew(C2(:,3))*(o_6 - o_2); C2(:,3)];
    J3 = [ skew(C3(:,3))*(o_6 - o_3); C3(:,3)];
    J4 = [ skew(C4(:,3))*(o_6 - o_4); C4(:,3)];
    J5 = [ skew(C5(:,3))*(o_6 - o_5); C5(:,3)];
    
    J = [J0 J1 J2 J3 J4 J5];
end