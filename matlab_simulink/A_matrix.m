function A = A_matrix(x)
    % part of the non-linear model:
    % A*xdd + B*xd + damping*xd + C = U
    
    global alpha J M m r I l;    
    phi = x(1);
    theta = x(2);
    % phi_dot = x(3);
    % theta_dot= x(4);
    a11 = J + (M + m)*r^2;
    a12 = J + (M + m)*r^2 + M*l*r*cos(theta + alpha);
    a21 = J + (M + m)*r^2 + M*l*r*cos(theta+alpha);
    a22 = J + (M + m)*r^2 + 2*M*l*r*cos(theta+alpha) + I + M*l^2;
    A = [a11, a12; a21, a22];
end