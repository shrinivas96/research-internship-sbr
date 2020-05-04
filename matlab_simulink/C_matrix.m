function C = C_matrix(x)
    % part of the non-linear model
    % A*xdd + B*xd + damping*xd + C = U
    
    global M m g r alpha M g l;
    phi = x(1);
    theta = x(2);
    c11 = (M+m)*g*r*sin(alpha);
    C = [c11; c11 + M*g*l*sin(theta)];
end
