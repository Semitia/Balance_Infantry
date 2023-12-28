phi_1 = 2.8;
phi_4 = 0;

L(1) = 0.15;
L(2) = 0.3;
L(3) = 0.3;
L(4) = 0.15;
L(5) = 0.15;

y = [];

index_1 = 1;

for i = 1.8:0.1:3.3
    index_2 = 1;

    for j = -0.2:0.1:1.3
        phi = forward_solve(L, i, 0);
        %     J_11_ = subs(J_11, theta_1, phi(1));
        %     J_11_ = subs(J_11_, theta_2, phi(2));
        %     J_11_ = subs(J_11_, theta_3, phi(3));
        %     J_11_ = subs(J_11_, theta_4, phi(4));

        J_14_ = subs(J_14, theta_1, phi(1));
        J_14_ = subs(J_14_, theta_2, phi(2));
        J_14_ = subs(J_14_, theta_3, phi(3));
        J_14_ = subs(J_14_, theta_4, phi(4));
        y(index_1, index_2) = eval(J_14_);
        index_2 = index_2 + 1;
        %J_11_1 = subs(J_11, theta_5, phi(5));
        %eval(J_11_)
    end

    index_1 = index_1 + 1;
end

% x = 1.8:0.1:3.3;
mesh(1.8:0.1:3.3, -0.2:0.1:1.3, y)
scatter(x, y)

%phi = forward_solve(L, phi_1, phi_4);

% J_11_ = subs(J_11, theta_1, phi(1));
% J_11_ = subs(J_11_, theta_2, phi(2));
% J_11_ = subs(J_11_, theta_3, phi(3));
% J_11_ = subs(J_11_, theta_4, phi(4));

% %J_11_1 = subs(J_11, theta_5, phi(5));
% eval(J_11_)

% J_14_ = subs(J_14, theta_1, phi(1));
% J_14_ = subs(J_14_, theta_2, phi(2));
% J_14_ = subs(J_14_, theta_3, phi(3));
% J_14_ = subs(J_14_, theta_4, phi(4));
% eval(J_14_)
%
% J_141_ = subs(J_141, theta_1, phi(1));
% J_141_ = subs(J_141_, theta_2, phi(2));
% J_141_ = subs(J_141_, theta_3, phi(3));
% J_141_ = subs(J_141_, theta_4, phi(4));
% eval(J_141_)
%
% J_144_ = subs(J_144, theta_1, phi(1));
% J_144_ = subs(J_144_, theta_2, phi(2));
% J_144_ = subs(J_144_, theta_3, phi(3));
% J_144_ = subs(J_144_, theta_4, phi(4));
% eval(J_144_)
%
% J_111_ = subs(J_111, theta_1, phi(1));
% J_111_ = subs(J_111_, theta_2, phi(2));
% J_111_ = subs(J_111_, theta_3, phi(3));
% J_111_ = subs(J_111_, theta_4, phi(4));
% eval(J_111_)
%
% J_114_ = subs(J_114, theta_1, phi(1));
% J_114_ = subs(J_114_, theta_2, phi(2));
% J_114_ = subs(J_114_, theta_3, phi(3));
% J_114_ = subs(J_114_, theta_4, phi(4));
% eval(J_114_)
%
% J_441_ = subs(J_441, theta_1, phi(1));
% J_441_ = subs(J_441_, theta_2, phi(2));
% J_441_ = subs(J_441_, theta_3, phi(3));
% J_441_ = subs(J_441_, theta_4, phi(4));
% eval(J_441_)
%
% J_444_ = subs(J_444, theta_1, phi(1));
% J_444_ = subs(J_444_, theta_2, phi(2));
% J_444_ = subs(J_444_, theta_3, phi(3));
% J_444_ = subs(J_444_, theta_4, phi(4));
% eval(J_441_)
