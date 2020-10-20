function [theta, delta] = kahanP3(s,t,u,v)
    s_hat = s/norm(s);
    t_hat = t/norm(t);
    u_hat = u/norm(u);
    v_hat = v/norm(u);
    alpha = (1/((norm(cross(s_hat, t_hat)))^2))*(u_hat'*s_hat - (s_hat'*t_hat)*(v_hat'*t_hat));
    beta = (1/((norm(cross(s_hat, t_hat)))^2))*(v_hat'*t_hat - (s_hat'*t_hat)*(u_hat'*s_hat));
    z = alpha*s_hat + beta*t_hat;
    cond = 1-(norm(z)^2);
    if(cond < 0)
        theta = NaN;
        delta = NaN;
    end
    
    w_hatPlus = z + ((sqrt(cond)/norm(cross(s_hat, t_hat)))*(cross(s_hat, t_hat)));
    w_hatMinus = z - ((sqrt(cond)/norm(cross(s_hat, t_hat)))*(cross(s_hat, t_hat)));
    theta = [kahanP2(s_hat, u_hat, w_hatPlus); kahanP2(s_hat, u_hat, w_hatMinus)];  
    delta = [kahanP2(t_hat, v_hat, w_hatPlus); kahanP2(t_hat, v_hat, w_hatMinus)];
end