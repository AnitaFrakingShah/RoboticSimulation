function theta = kahanP1(s,t)
    s_hat = s/norm(s);
    t_hat = t/norm(t);
    theta = 2*(atan(norm(s_hat-t_hat)/norm(s_hat+t_hat)));
end
