function xdot = nl_eq_function(t, x)
    global K;
    phi = x(1);
    theta = x(2);
    phi_dot = x(3);
    theta_dot = x(4);
    
    tau_ext = -(K*x);
    A = A_matrix([phi; theta]);
    A_inv = inv(A);
    B_xd = B_matrix([phi; theta; phi_dot; theta_dot]);
    damp = damping_matrix([phi; theta]);
    const = C_matrix([phi; theta]);
    feedback = [tau_ext; 0];

    
    var1 = A_inv*(feedback - (B_xd * [phi_dot; theta_dot]) - (damp * [phi_dot; theta_dot]) - const);
    xdot(1) = x(3);
    xdot(2) = x(4); 
    xdot(3) = [1 0]*var1;
    xdot(4) = [0 1]*var1;
    xdot = xdot';
end
% [K,S,E] = lqr(lin_A, lin_B , eye(4),R,N)