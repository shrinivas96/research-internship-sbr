function B = B_matrix(x)
    % part of the non-linear model
    % A*xdd + B*xd + damping*xd + C = U
    
    global alpha n1 M l r;
    % phi = x(1);
    theta = x(2);
    % phi_dot = x(3);
    theta_dot= x(4);
    n1 = -1*(M*l*r*theta_dot*sin(theta + alpha));
    B = [0, n1; 0, n1];
end
