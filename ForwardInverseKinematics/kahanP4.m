function theta = kahanP4(a,b,c)
    if(a+b>c && c > abs(a-b))
        top = ((a+b)^2)-(c^2);
        bottom = (c^2 - (a-b)^2);
        theta = abs(2* atan(sqrt(top/bottom)));
    else
        theta = NaN;
    end
end