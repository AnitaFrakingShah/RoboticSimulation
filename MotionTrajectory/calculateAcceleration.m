function [acceleration] = calculateAcceleration(change_o, change_theta)
    % If we recall we can calculate displacement as D = V*t + (a*t^2)/2
    % based on our behavioral graphs we know that our V at time 0 should be
    % 0, our acceleration should be constant for 0 -> 0.5 and then
    % deceleration should be constant for 0.5 -> 1 
    
    % Thus our D = 0 + (1/2 * (a*(0.5)^2)) + (0 + a*0.5)*0.5 - (1/2 *
    % (a*(0.5)^2)) ==> D = 0.5*a 
    % Thus our a = D/0.25
    timeSquared = (0.5 - .005) * (0.5 + .005);
    a1 = change_o(1) / timeSquared;
    a2 = change_o(2) / timeSquared;
    a3 = change_o(3) / timeSquared;
    a4 = change_theta(1) / timeSquared;
    a5 = change_theta(2) / timeSquared;
    a6 = change_theta(3) / timeSquared;
    acceleration = [a1; a2; a3; a4; a5; a6];
    
    %% Divide this acceleration by the number of time steps we are going to be taking
    %% i.e. 200 timesteps (frequency of 200 hz).
    acceleration = acceleration / 200;
    
end