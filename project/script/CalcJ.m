function J = CalcJ(phi, l, phi_0, l_0)
    sigma_1 = sin(phi(2) -phi(3));
    sigma_2 = sin(phi(3) - phi(4));
    sigma_3 = sin(phi(1) - phi(2));

    J = zeros(2, 2);
    J(1, 1) = -l(1) * cos(phi_0 - phi(3)) * sigma_3 / l_0 / sigma_1;
    J(1, 2) = -l(1) * sin(phi_0 - phi(3)) * sigma_3 / sigma_1;
    J(2, 1) = -l(4) * cos(phi_0 - phi(2)) * sigma_2 / l_0 / sigma_1;
    J(2, 2) = -l(4) * sin(phi_0 - phi(2)) * sigma_2 / sigma_1;
end
