
    syms x(t) theta(t) fai(t) T T_p N P N_M P_M
    syms x1 x2 x3 x4 x5 L

    syms phi_ theta_ x_ phi_dot theta_dot x_dot
    syms phi_dot_2 theta_dot_2 x_dot_2

    % params 可以完全确认的参数
    g = 9.793383;
    %R = 0.0675; % wheel r
    R = 0.11;
    m_w = 1.28; % wheel wieght
    m_p = (0.134+ 0.237) * 2; %摆杆质量
    L_M = L * 0.5; % 摆杆重心到驱动轮轴据距离？？
    M = 13.7; % body weight
    %L = 0.1117;     % 杆重心到驱动轮轴距离 ？？

    %从机械处获得
    I_M = 0.2884626; % 机体绕质心转动惯量
    l = 0.05; % 机体中心离其转轴距离   ？？
 
    %暂时不知道怎么获取
    I_w = 0.015488; % 通过系统辨识获取
    I_p = 0.045; % 摆杆绕质心转动惯量
    
    eq3 = diff(x, t, 2) == (T - N * R) / (I_w / R + m_w * R);
    %eq4 = N - N_M == m_p * diff(x + L * sin(theta),t,2);
    %eq5 = P - P_M - m_p * g == m_p * diff(L * cos(theta),t,2);
    eq6 = I_p * diff(theta, t, 2) == (P * L + P_M * L_M) * sin(theta) - (N * L + N_M * L_M) * cos(theta) - T + T_p;
    %eq7 = N_M == M * diff(x+(L +L_M)*sin(theta) - l * sin(fai),t,2);
    %eq8 = P_M - M * g == M * diff((L+L_M)*cos(theta) + l * cos(fai),t,2);
    eq9 = I_M * diff(fai, t, 2) == T_p + N_M * l * cos(fai) + P_M * l * sin(fai);

    eq6 = subs(eq6, P, m_p * diff(L * cos(theta), t, 2) + m_p * g + P_M);
    eq6 = subs(eq6, P_M, M * diff((L + L_M) * cos(theta) + l * cos(fai), t, 2) + M * g);
    eq9 = subs(eq9, P_M, M * diff((L + L_M) * cos(theta) + l * cos(fai), t, 2) + M * g);

    eq6 = subs(eq6, N, m_p * diff(x + L * sin(theta), t, 2) + N_M);
    eq6 = subs(eq6, N_M, M * diff(x + (L +L_M) * sin(theta) - l * sin(fai), t, 2));
    eq9 = subs(eq9, N_M, M * diff(x + (L +L_M) * sin(theta) - l * sin(fai), t, 2));

    eq3 = subs(eq3, N, m_p * diff(x + L * sin(theta), t, 2) + N_M);
    eq3 = subs(eq3, N_M, M * diff(x + (L +L_M) * sin(theta) - l * sin(fai), t, 2));

    eq6 = simplify(eq6);
    eq9 = simplify(eq9);

    eq6 = subs(eq6, diff(theta, t, 2), theta_dot_2);
    eq6 = subs(eq6, diff(fai, t, 2), phi_dot_2);
    eq6 = subs(eq6, diff(x, t, 2), x_dot_2);

    eq3 = subs(eq3, diff(theta, t, 2), theta_dot_2);
    eq3 = subs(eq3, diff(fai, t, 2), phi_dot_2);
    eq3 = subs(eq3, diff(x, t, 2), x_dot_2);

    eq9 = subs(eq9, diff(theta, t, 2), theta_dot_2);
    eq9 = subs(eq9, diff(fai, t, 2), phi_dot_2);
    eq9 = subs(eq9, diff(x, t, 2), x_dot_2);

    last = solve(eq3, eq6, eq9, theta_dot_2, x_dot_2, phi_dot_2);

    last.theta_dot_2 = subs(last.theta_dot_2, theta(t), theta_);
    last.theta_dot_2 = subs(last.theta_dot_2, fai(t), phi_);
    last.theta_dot_2 = subs(last.theta_dot_2, x(t), x_);

    last.x_dot_2 = subs(last.x_dot_2, theta(t), theta_);
    last.x_dot_2 = subs(last.x_dot_2, fai(t), phi_);
    last.x_dot_2 = subs(last.x_dot_2, x(t), x_);

    last.phi_dot_2 = subs(last.phi_dot_2, theta(t), theta_);
    last.phi_dot_2 = subs(last.phi_dot_2, fai(t), phi_);
    last.phi_dot_2 = subs(last.phi_dot_2, x(t), x_);

    last.theta_dot_1 = theta_dot;
    last.x_dot_1 = x_dot;
    last.phi_dot_1 = phi_dot;

    state_vector = [theta_, theta_dot, x_, x_dot, phi_, phi_dot];
    u = [T, T_p];
    f = [last.theta_dot_1, last.theta_dot_2, last.x_dot_1, last.x_dot_2, last.phi_dot_1, last.phi_dot_2];

    A = jacobian(f, state_vector);
    B = jacobian(f, u);

    A = subs(A, {theta_, theta_dot, x_, x_dot, phi_, phi_dot,T,T_p}, {0, 0, x, 0, 0, 0,0,0});
    B = subs(B, {theta_, theta_dot, x_, x_dot, phi_, phi_dot,T,T_p}, {0, 0, x, 0, 0, 0,0,0});

    L_base = 0.10;
    L_step = 0.01;
    L_curve_size = 20;

    % 得到A，B之后
    Q = zeros(6, 6);
    ratio = 100;  % 表示状态的重要程度
    Q(1, 1) = 0.01 * ratio;
    Q(2, 2) = 0.01 * ratio;
    Q(3, 3) = 5 * ratio;
    Q(4, 4) = 1 * ratio;
    Q(5, 5) = 50 * ratio;
    Q(6, 6) = 0.01 * ratio;
    R = zeros(2, 2);
    R(1, 1) = 1;
    R(2, 2) = 0.25;
    
%     K = lqr(eval(A), eval(B), Q, R);
%     
%     for i = 1:2
%         for j = 1:6
%             fprintf("balance_infantry->K[%d][%d] = %f; \n",i-1,j-1,K(i,j));
%         end
%     end
    A_o = [];
    B_o = [];

    set = zeros(L_curve_size, 2, 6);
    x_curve = zeros(L_curve_size, 1);

    for i = 1:20
        x_curve(i) = L_base + i * L_step;
        A_ = eval(subs(A, {L}, {L_base + i * L_step}));
        B_ = eval(subs(B, {L}, {L_base + i * L_step}));
        if i == 5
            A_o = A_;
            B_o = B_;
        end
        K = lqr(A_, B_, Q, R)
        set(i, :, :) = K;
    end

    fprintf("float K_params[12][3] = {");

    for i = 1:2
        for j = 1:6
            fprintf("{");

            y_curve = set(:, i, j);
            P = polyfit(x_curve, y_curve, 2);
            %P
            fprintf("%f,%f,%f },\n",P(1),P(2),P(3));
        end
    end
    fprintf("};\n");
    
%     B_o = B_o * 0.001;
%     B_temp = [0 0 0.5 * 0.001 * 0.001 0.001 0 0].';
%     B_o = [B_o B_temp;0,0,0]
%     
%     A_o = [A_o , A_o(:,5);zeros(1,7)];
%     
    %A_o2 = expm(A_o * 0.001)
    
    %sym tt
    %int(expm(A_o * tt) * )
%     
%     % 利用此求解L矩阵
%     C_raw = [1,zeros(1,6);zeros(1,2),1,zeros(1,4);zeros(1,3),1,zeros(1,3);zeros(1,4),1,zeros(1,2)]
%     
%     L_raw = place(A_raw.',C_raw.',[-5 -6 -7 -8 -9 -10 -11].');
%     L_o = L_raw * 0.001;
%     L_o = L_o.'

