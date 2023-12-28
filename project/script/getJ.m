syms phi_1(t) phi_2(t) phi_3(t) phi_4(t) phi_0(t)
syms phi_dot_1 phi_dot_2 phi_dot_3 phi_dot_4
syms l_1 l_2 l_3 l_4 l_5 l_0

x_B = l_1 * cos(phi_1);
y_B = l_1 * sin(phi_1);
x_C = x_B + l_2 * cos(phi_2);
y_C = y_B + l_2 * sin(phi_2);
x_D = l_5 + l_4 * cos(phi_4);
y_D = l_4 * sin(phi_4);
x_dot_B = diff(x_B, t);
x_dot_C = diff(x_C, t);
x_dot_D = diff(x_D, t);
y_dot_B = diff(y_B, t);
y_dot_C = diff(y_C, t);
y_dot_D = diff(y_D, t);

phi_dot_2 = ((x_dot_D - x_dot_B) * cos(phi_3) + (y_dot_D - y_dot_B) * sin(phi_3)) / (l_2 * sin(phi_3 - phi_2));
phi_dot_3 = ((x_dot_D - x_dot_B) * cos(phi_2) + (y_dot_D - y_dot_B) * sin(phi_2)) / (l_3 * sin(phi_3 - phi_2));

x_dot_C = subs(x_dot_C, diff(phi_2, t), phi_dot_2);
x_dot_C = subs(x_dot_C, {diff(phi_1, t), diff(phi_3, t), diff(phi_4, t)}, {phi_dot_1, phi_dot_3, phi_dot_4});
y_dot_C = subs(y_dot_C, diff(phi_2, t), phi_dot_2);
y_dot_C = subs(y_dot_C, {diff(phi_1, t), diff(phi_3, t), diff(phi_4, t)}, {phi_dot_1, phi_dot_3, phi_dot_4});

x_dot = [x_dot_C; y_dot_C];
q_dot = [phi_dot_1; phi_dot_4];
x_dot = collect(x_dot, q_dot);
x_dot = simplify(x_dot);
x_dot = collect(x_dot, q_dot);

J = jacobian(x_dot, q_dot);
J = simplify(J);
rotate = [cos(phi_0 - pi / 2) -sin(phi_0 - pi / 2); sin(phi_0 - pi / 2) cos(phi_0 - pi / 2)];
map = [-1 / l_0 0; 0, 1];
J = simplify(J.' * rotate * map);
