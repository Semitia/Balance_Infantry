Q = [10,0,0,0; 0,1,0,0; 0,0,1,0; 0,0,0,1];
R = 0.1;
A = [0,1,0,0; 432.353,0,0,0; 0,0,0,1; -4.9,0,0,0];
B = [0; -7.353; 0; 0.25];
C = [0,0,1,0];
D = 0;

sys = ss(A,B,C,D);
[K,~,~] = lqr(sys, Q,R);
disp(K);