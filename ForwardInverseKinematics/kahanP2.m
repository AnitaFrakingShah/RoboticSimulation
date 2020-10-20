function theta = kahanP2(s,u,w)
    s_hat = s/norm(s);
    u_hat = u/norm(u);
    w_hat = w/norm(w);
    a = (s_hat' * u_hat);
    b = (s_hat' * w_hat);
    Cfirst = abs(a - b);
    Csecond = 1000*eps("single");
    if(Cfirst < 1)
        theta = 2*atan(norm(cross(s_hat, (u_hat - w_hat)))/norm(cross(s_hat, (u_hat + w_hat))));
        a = w_hat' * cross(s_hat, u_hat);
        b = w_hat' * cross(s_hat, (u_hat - w_hat));
        Cfirst = abs(a - b);
        if(Cfirst < Csecond && a < 0 && b < 0)
            theta = -theta;
        end
    else
        theta = NaN;
    end
end