%% ========================================================================
%  CHUONG TRINH TINH MA TRAN DH (Denavit-Hartenberg)
%  Tinh tung ma tran A_i va ma tran bien doi tong hop T = A1*A2*...*An
%  ========================================================================
clc; clear; close all;

%% Khai bao bien symbolic
syms q0 q1 q2 q3 l0 real

%% ========================================================================
%  BANG THAM SO DH
%  ========================================================================
%  Cong thuc ma tran A_i:
%  A_i = [cos(theta_i)  -sin(theta_i)*cos(alpha_i)   sin(theta_i)*sin(alpha_i)  a_i*cos(theta_i)
%         sin(theta_i)   cos(theta_i)*cos(alpha_i)  -cos(theta_i)*sin(alpha_i)  a_i*sin(theta_i)
%         0              sin(alpha_i)                 cos(alpha_i)               d_i
%         0              0                            0                          1              ]
%  ========================================================================
%
%  Bang DH:
%  | Khop i |  d_i  | theta_i |  a_i  | alpha_i |
%  |--------|-------|---------|-------|---------|
%  |   1    |  l0   |   q0    |   0   |  pi/2   |
%  |   2    |   0   |   q1    |   9   |    0    |
%  |   3    |   0   |   q2    |   9   |    0    |
%  |   4    |   0   |   q3    |   3   |    0    |
%  ========================================================================

% Tham so DH: [d_i, theta_i, a_i, alpha_i]
DH = [  l0,  q0,  0,  pi/2;
         0,  q1,  9,     0;
         0,  q2,  9,     0;
         0,  q3,  3,     0  ];

n = size(DH, 1);  % So khop

%% ========================================================================
%  HAM TINH MA TRAN A_i TU THAM SO DH
%  ========================================================================
function A = DH_matrix(d, theta, a, alpha)
    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

%% ========================================================================
%  TINH TUNG MA TRAN A_i
%  ========================================================================
fprintf('================================================================\n');
fprintf('         TINH TOAN MA TRAN BIEN DOI DH CHO ROBOT 4 KHOP\n');
fprintf('================================================================\n\n');

% Luu cac ma tran A_i
A = cell(1, n);

for i = 1:n
    d_i     = DH(i, 1);
    theta_i = DH(i, 2);
    a_i     = DH(i, 3);
    alpha_i = DH(i, 4);
    
    % Tinh ma tran A_i
    A{i} = simplify(DH_matrix(d_i, theta_i, a_i, alpha_i));
    
    % Hien thi ma tran A_i
    fprintf('----------------------------------------------------------------\n');
    fprintf('  MA TRAN A%d (Khop %d): d=%s, theta=%s, a=%s, alpha=%s\n', ...
            i, i, char(d_i), char(theta_i), char(a_i), char(alpha_i));
    fprintf('----------------------------------------------------------------\n');
    disp(A{i});
    fprintf('\n');
end

%% ========================================================================
%  TINH MA TRAN BIEN DOI TONG HOP T = A1 * A2 * A3 * A4
%  ========================================================================
fprintf('================================================================\n');
fprintf('  TINH MA TRAN BIEN DOI TONG HOP T = A1 * A2 * A3 * A4\n');
fprintf('================================================================\n\n');

% Tinh tich luy tung buoc
T = eye(4);  % Ma tran don vi 4x4

for i = 1:n
    T = T * A{i};
    T = simplify(T);
    
    % Hien thi ket qua trung gian
    if i > 1
        fprintf('----------------------------------------------------------------\n');
        fprintf('  T(0->%d) = A1', i);
        for j = 2:i
            fprintf(' * A%d', j);
        end
        fprintf('\n');
        fprintf('----------------------------------------------------------------\n');
        disp(T);
        fprintf('\n');
    end
end

%% ========================================================================
%  HIEN THI KET QUA CUOI CUNG
%  ========================================================================
fprintf('================================================================\n');
fprintf('  MA TRAN BIEN DOI TONG HOP CUOI CUNG: T(0->4)\n');
fprintf('================================================================\n\n');

T_final = simplify(T);
disp(T_final);

%% ========================================================================
%  TRICH XUAT VI TRI VA HUONG CUA END-EFFECTOR
%  ========================================================================
fprintf('\n================================================================\n');
fprintf('  VI TRI END-EFFECTOR (Px, Py, Pz)\n');
fprintf('================================================================\n\n');

Px = simplify(T_final(1, 4));
Py = simplify(T_final(2, 4));
Pz = simplify(T_final(3, 4));

fprintf('  Px = %s\n\n', char(Px));
fprintf('  Py = %s\n\n', char(Py));
fprintf('  Pz = %s\n\n', char(Pz));

fprintf('================================================================\n');
fprintf('  MA TRAN QUAY R (3x3)\n');
fprintf('================================================================\n\n');

R = simplify(T_final(1:3, 1:3));
disp(R);

fprintf('\n================================================================\n');
fprintf('  HOAN THANH TINH TOAN!\n');
fprintf('================================================================\n');
