function phi = forward_solve(L, phi_1, phi_4)
    phi(1) = phi_1;
    phi(4) = phi_4;
    x_B = L(1) * cos(phi(1));
    y_B = L(1) * sin(phi(1));
    x_D = L(4) * cos(phi(4)) + L(5);
    y_D = L(4) * sin(phi(4));
    B_D_distance_2 = (x_D - x_B)^ 2 + (y_D - y_B)^2;
    A_0 = 2 * L(2) * (x_D - x_B);
    B_0 = 2 * L(2) * (y_D - y_B);
    C_0 = L(2)^2 + B_D_distance_2 - L(3)^ 2;

    temp = (B_0 + sqrt(A_0^ 2 + B_0^ 2 - C_0^ 2));
    phi(2) = 2 * atan2(temp, A_0 + C_0);

    x_C = x_B + L(2) * cos(phi(2));
    y_C = y_B + L(2) * sin(phi(2));

    phi(3) = atan2(y_C - y_D, x_C - x_D);

    top_half_x = L(5) / 2;
    x_C = x_C - top_half_x;

    %L(0) = sqrt(x_C * x_C + y_C * y_C);

    %phi(0) = atan2(y_C, x_C);
end
