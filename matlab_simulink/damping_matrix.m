function D = damping_matrix(x)
    % part of the non-linear model:
    % A*xdd + B*xd + damping*xd + C = U
    
    global r b c;
    phi = x(1);
    theta = x(2);
    d1 = b/r;
    d21 = 0;
    d22 = c;
    D = [d1, d1; d21, d22];
end