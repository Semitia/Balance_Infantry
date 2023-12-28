syms t
syms theta_1(t) theta_4(t)
syms theta_2(theta_1, theta_4) theta_3(theta_1, theta_4)
syms theta_5(theta_1, theta_4)
syms diff_theta_dot_2_theta_dot_1
syms diff_theta_dot_2_theta_dot_4
syms diff_theta_dot_3_theta_dot_1
syms diff_theta_dot_3_theta_dot_4
syms diff_theta_2_theta_1 diff_theta_2_theta_4
syms diff_theta_3_theta_1 diff_theta_3_theta_4

syms theta_1_dot_1 theta_1_dot_2 theta_4_dot_1 theta_4_dot_2
syms k_1(t) k_2 k_3 k_5 k_4(t)
%diff_theta_2_theta_1 = -(l_1 * sin(theta_1 - theta_3))/(l_2 * sin(theta_2 - theta_3))
%diff_theta_2_theta_4 = -(l_4 * sin(theta_3 - theta_4))/(l_2 * sin(theta_2 - theta_3))
%diff_theta_3_theta_1 = -(l_1 * sin(theta_1 - theta_2))/(l_3 * sin(theta_2 - theta_3))
%diff_theta_3_theta_4 = -(l_4 * sin(theta_2 - theta_4))/(l_3 * sin(theta_2 - theta_3))
%diff_theta_dot_2_theta_dot_1 = -(l_1 * sin(theta_1 - theta_3)) / (l_2 *
%sin(theta_2 - theta_3))
%diff_theta_dot_2_theta_dot_4 = -(l_4 * sin(theta_3 - theta_4)) / (l_2 *
%sin(theta_2 - theta_3))
%diff_theta_dot_3_theta_dot_1 = -(l_1 * sin(theta_1 - theta_2)) / (l_3 *
%sin(theta_2 - theta_3))
%diff_theta_dot_3_theta_dot_4 = -(l_4 * sin(theta_2 - theta_4)) / (l_3 *
%sin(theta_2 - theta_3))
m_1 = 0.3;
m_2 = 0.6;
m_3 = 0.6;
m_4 = 0.3;
%m_5 = 0.2;

l_1 = 0.15;
l_2 = 0.3;
l_3 = 0.3;
l_4 = 0.15;
l_5 = 0.15;

J_1 = 1e-3;
J_2 = 4e-3;
J_3 = 4e-3;
J_4 = 1e-3;
%J_5 = 0.01;

J_11 = m_1 * (l_1 / 2)^2;
J_11 = J_11 + J_1;
J_11 = J_11 + m_2 * (l_1 * sin(theta_1) + l_2 / 2 * sin(theta_2) * diff_theta_dot_2_theta_dot_1)^2;
J_11 = J_11 + m_2 * (l_1 * cos(theta_1) + l_2 / 2 * cos(theta_2) * diff_theta_dot_2_theta_dot_1)^2;
J_11 = J_11 + J_2 * diff_theta_dot_2_theta_dot_1^2;
J_11 = J_11 + m_3 * (l_3 / 2)^2 * (diff_theta_dot_3_theta_dot_1)^2;
J_11 = J_11 + J_3 * diff_theta_dot_3_theta_dot_1^2;
J_11 = subs(J_11, diff_theta_dot_2_theta_dot_1, - (l_1 * sin(theta_1 - theta_3)) / (l_2 * sin(theta_2 - theta_3)));
J_11 = subs(J_11, diff_theta_dot_3_theta_dot_1, - (l_1 * sin(theta_1 - theta_2)) / (l_3 * sin(theta_2 - theta_3)));

J_11 = simplify(J_11);

J_44 = m_4 * (l_4 / 2)^2;
J_44 = J_44 + J_4;
J_44 = J_44 + m_3 * (l_4 * sin(theta_4) + l_3 / 2 * sin(theta_3) * diff_theta_dot_3_theta_dot_4)^2;
J_44 = J_44 + m_3 * (l_4 * cos(theta_4) + l_3 / 2 * cos(theta_3) * diff_theta_dot_3_theta_dot_4)^2;
J_44 = J_44 + J_3 * diff_theta_dot_3_theta_dot_4^2;
J_44 = J_44 + m_2 * (l_2 / 2)^2 * (diff_theta_dot_2_theta_dot_4)^2;
J_44 = J_44 + J_2 * diff_theta_dot_2_theta_dot_4^2;
J_44 = subs(J_44, diff_theta_dot_2_theta_dot_4, - (l_4 * sin(theta_3 - theta_4)) / (l_2 * sin(theta_2 - theta_3)));
J_44 = subs(J_44, diff_theta_dot_3_theta_dot_4, - (l_4 * sin(theta_2 - theta_4)) / (l_3 * sin(theta_2 - theta_3)));
J_44 = simplify(J_44);

J_14 = m_2 * (l_2 / 2)^2 * diff_theta_dot_2_theta_dot_1 * diff_theta_dot_2_theta_dot_4;
J_14 = J_14 + J_2 * diff_theta_dot_2_theta_dot_1 * diff_theta_dot_2_theta_dot_4;
J_14 = J_14 + m_2 * l_1 * l_2 / 2 * sin(theta_1) * sin(theta_2) * diff_theta_dot_2_theta_dot_4;
J_14 = J_14 + m_3 * (l_3 / 2)^2 * diff_theta_dot_3_theta_dot_1 * diff_theta_dot_3_theta_dot_4;
J_14 = J_14 + J_2 * diff_theta_dot_3_theta_dot_1 * diff_theta_dot_3_theta_dot_4;
J_14 = J_14 + m_3 * l_4 * l_3 / 2 * sin(theta_4) * sin(theta_3) * diff_theta_dot_3_theta_dot_1;
J_14 = J_14 + m_3 * l_4 * l_3 / 2 * cos(theta_4) * cos(theta_3) * diff_theta_dot_3_theta_dot_1;
J_14 = subs(J_14, diff_theta_dot_3_theta_dot_4, - (l_4 * sin(theta_2 - theta_4)) / (l_3 * sin(theta_2 - theta_3)));
J_14 = subs(J_14, diff_theta_dot_2_theta_dot_4, - (l_4 * sin(theta_3 - theta_4)) / (l_2 * sin(theta_2 - theta_3)));
J_14 = subs(J_14, diff_theta_dot_2_theta_dot_1, - (l_1 * sin(theta_1 - theta_3)) / (l_2 * sin(theta_2 - theta_3)));
J_14 = subs(J_14, diff_theta_dot_3_theta_dot_1, - (l_1 * sin(theta_1 - theta_2)) / (l_3 * sin(theta_2 - theta_3)));

J_14 = simplify(J_14);

%theta_1_dot_1 = diff(theta_1,t,1)
%theta_1_dot_2 = diff(theta_1,t,2)
%theta_4_dot_1 = diff(theta_4,t,1)
%theta_4_dot_2 = diff(theta_4,t,2)

% T_1 = J_11 * theta_1_dot_2 + J_14 * theta_4_dot_2 + 0.5 * diff(J_11,theta_1) * theta_1_dot_1^2;
% T_1 = T_1 + diff(J_11,theta_4) * theta_1_dot_1 * theta_4_dot_1;
% T_1 = T_1 + (diff(J_14,theta_4) - 0.5 * diff(J_44,theta_1)) * theta_4_dot_1^2;
% T_1 = subs(T_1,diff(theta_2,theta_1),-(l_1 * sin(theta_1 - theta_3))/(l_2 * sin(theta_2 - theta_3)));
% T_1 = subs(T_1,diff(theta_2,theta_4),-(l_4 * sin(theta_3 - theta_4))/(l_2 * sin(theta_2 - theta_3)));
% T_1 = subs(T_1,diff(theta_3,theta_1),-(l_1 * sin(theta_1 - theta_2))/(l_3 * sin(theta_2 - theta_3)));
% T_1 = subs(T_1,diff(theta_3,theta_4),-(l_4 * sin(theta_2 - theta_4))/(l_3 * sin(theta_2 - theta_3)));
% T_1 = subs(T_1,theta_2,k_2);
% T_1 = subs(T_1,theta_3,k_3);
% T_1 = subs(T_1,theta_5,k_5);
% T_1 = subs(T_1,theta_4,k_4);
% T_1 = subs(T_1,theta_1,k_1);
%T_1 = collect(T_1);
%T_1 = simplify(T_1);

% T_2 = J_14 * theta_1_dot_2 + J_44 * theta_4_dot_2 + 0.5 * diff(J_44,theta_4) * theta_4_dot_1^2;
% T_2 = T_2 + diff(J_44,theta_1) * theta_1_dot_1 * theta_4_dot_1;
% T_2 = T_2 + (diff(J_14,theta_1) - 0.5 * diff(J_11,theta_4)) * theta_1_dot_1^2;
J_141 = diff(J_14, theta_1);
J_141 = subs(J_141, diff(theta_2, theta_1), - (l_1 * sin(theta_1 - theta_3)) / (l_2 * sin(theta_2 - theta_3)));
J_141 = subs(J_141, diff(theta_3, theta_1), - (l_1 * sin(theta_1 - theta_2)) / (l_3 * sin(theta_2 - theta_3)));
J_141 = simplify(J_141);

J_144 = diff(J_14, theta_4);
J_144 = subs(J_144, diff(theta_2, theta_4), - (l_4 * sin(theta_3 - theta_4)) / (l_2 * sin(theta_2 - theta_3)));
J_144 = subs(J_144, diff(theta_3, theta_4), - (l_4 * sin(theta_2 - theta_4)) / (l_3 * sin(theta_2 - theta_3)));
J_144 = simplify(J_144);

J_441 = diff(J_44, theta_1);
J_441 = subs(J_441, diff(theta_2, theta_1), - (l_1 * sin(theta_1 - theta_3)) / (l_2 * sin(theta_2 - theta_3)));
J_441 = subs(J_441, diff(theta_3, theta_1), - (l_1 * sin(theta_1 - theta_2)) / (l_3 * sin(theta_2 - theta_3)));
J_441 = simplify(J_441);

J_444 = diff(J_44, theta_4);
J_444 = subs(J_444, diff(theta_2, theta_4), - (l_4 * sin(theta_3 - theta_4)) / (l_2 * sin(theta_2 - theta_3)));
J_444 = subs(J_444, diff(theta_3, theta_4), - (l_4 * sin(theta_2 - theta_4)) / (l_3 * sin(theta_2 - theta_3)));
J_444 = simplify(J_444);

J_111 = diff(J_11, theta_1);
J_111 = subs(J_111, diff(theta_2, theta_1), - (l_1 * sin(theta_1 - theta_3)) / (l_2 * sin(theta_2 - theta_3)));
J_111 = subs(J_111, diff(theta_3, theta_1), - (l_1 * sin(theta_1 - theta_2)) / (l_3 * sin(theta_2 - theta_3)));
J_111 = simplify(J_111);

J_114 = diff(J_11, theta_4);
J_114 = subs(J_114, diff(theta_2, theta_4), - (l_4 * sin(theta_3 - theta_4)) / (l_2 * sin(theta_2 - theta_3)));
J_114 = subs(J_114, diff(theta_3, theta_4), - (l_4 * sin(theta_2 - theta_4)) / (l_3 * sin(theta_2 - theta_3)));
J_114 = simplify(J_114);
