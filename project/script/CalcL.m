function [l_0, phi_0] = CalcL(l, phi)
    x_B = l(1) * cos(phi(1));
    y_B = l(1) * sin(phi(1));
    % x_D = l(5) + l(4) * cos(phi(4));
    % y_D = l(4) * sin(phi(4));
    % A_0 = 2 * l(2) * (x_D - x_B);
    % B_0 = 2 * l(2) * (y_D - y_B);
    % l_BD_2 = (x_D - x_B)^2 + (y_D - y_B)^2;
    % C_0 = l(2)^2 + l_BD_2 - l(3)^2;

    x_C = x_B + l(2) * cos(phi(2));
    y_C = y_B + l(2) * sin(phi(2));

    l_0 = sqrt((x_C - l(5) / 2)^2 + y_C^2);
    phi_0 = atan2(y_C, x_C - l(5) / 2);
end
