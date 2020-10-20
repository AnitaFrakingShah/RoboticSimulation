function [J_dot] = calculateJacobianInverse(q, q_dot)
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
    w1 = q_dot(1)*C0(:,3);
    
    %Forward Kinematics
    [T2,C12] = DH_homog(q(2), 0, 431.8, pi);
    C2 = C1*C12;
    endPosition = endPosition*T2;
    o_2 = endPosition(1:3,4);
    w2 = w1 + (q_dot(2)*C1(:,3));
    
    %Forward Kinematics
    [T3,C23] = DH_homog(q(3) + pi/2, -149.09, 20.32, pi/2);
    C3 = C2*C23;
    endPosition = endPosition*T3;
    o_3 = endPosition(1:3,4);
    w3 = w2 + (q_dot(3)*C2(:,3));
    
    %Forward Kinematics
    [T4,C34] = DH_homog(q(4), 433.1, 0, pi/2);
    C4 = C3*C34;
    endPosition = endPosition*T4;
    o_4 = endPosition(1:3,4);
    w4 = w3 + (q_dot(4)*C3(:,3));
    
    %Forward Kinematics
    [T5,C45] = DH_homog(q(5), 0, 0, -pi/2);
    C5 = C4*C45;
    endPosition = endPosition*T5;
    o_5 = endPosition(1:3,4);
    w5 = w4 + (q_dot(5)*C4(:,3));
    
    %Forward Kinematics
    [T6,C56] = DH_homog(q(6), 60, 0, 0);
    C6 = C5*C56;
    endPosition = endPosition*T6;
    o_6 = endPosition(1:3,4);
    w6 = w5 + (q_dot(6)*C5(:,3));
    
    % The top derivative based on equation 85 from chapter 6 dynamics gives
    % us a general idea of theta_i^2 * (k_i-1 x k_i-1 x) * (o_n - o_i)
    top0 = skew(C0(:,3))*skew(C0(:,3));
    top1 = skew(C1(:,3))*skew(C1(:,3));
    top2 = skew(C2(:,3))*skew(C2(:,3));
    top3 = skew(C3(:,3))*skew(C3(:,3));
    top4 = skew(C4(:,3))*skew(C4(:,3));
    top5 = skew(C5(:,3))*skew(C5(:,3));
    
    t1 = 2*(q_dot(1))*skew(C0(:,3))*skew(C1(:,3));
    
    t2 = 2*(q_dot(2))*skew(C1(:,3))*skew(C2(:,3));
    
    t3 = 2*(q_dot(3))*skew(C2(:,3))*skew(C3(:,3));
    
    t4 = 2*(q_dot(4))*skew(C3(:,3))*skew(C4(:,3));
    
    t5 = 2*(q_dot(5))*skew(C4(:,3))*skew(C5(:,3));
    
    J0 = [ q_dot(1)*(top0*(o_6 - o_0));  0;      0;    0];
    J1 = [ (t1*(o_6 - o_1) + q_dot(2)*(top1*(o_6 - o_1))); skew(w1)*C1(:,3)];
    J2 = [ (t2*(o_6 - o_2) + q_dot(3)*(top2*(o_6 - o_2))); skew(w2)*C2(:,3)];
    J3 = [ (t3*(o_6 - o_3) + q_dot(4)*(top3*(o_6 - o_3))); skew(w3)*C3(:,3)];
    J4 = [ (t4*(o_6 - o_4) + q_dot(5)*(top4*(o_6 - o_4))); skew(w4)*C4(:,3)];
    J5 = [ (t5*(o_6 - o_5) + q_dot(6)*(top5*(o_6 - o_5))); skew(w5)*C5(:,3)];
    
    J_dot = [J0 J1 J2 J3 J4 J5];
end