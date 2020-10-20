function [T, C] = DH_homog(theta1, d1, a1, alpha1)
    i=[1;0;0];
    j=[0;1;0];
    k=[0;0;1];
    a = [expm(theta1*skew(k)) zeros(3,1); zeros(1,3) 1];
    o = [eye(3) d1*k; zeros(1,3) 1];
    l = [eye(3) a1*i; zeros(1,3) 1];
    t = [expm(alpha1*skew(i)) zeros(3,1); zeros(1,3) 1];
    C = expm(theta1*skew(k)) * expm(alpha1*skew(i));
    T = a*o*l*t;
end