function sys = delftbike(v)

% Coefficient matrices;

load('JBike6MCK.mat', 'C1', 'M0', 'K2', 'K0')

a = -fliplr(eye(2)) + eye(2);
M0 = a .* M0;
C1 = C1 .* a;
K0 = K0 .* a;
K2 = K2 .* a;
Hfw = [0.9; 0.014408]; % dfx/dTq


O = zeros(2);
I = eye(2); % Some easy notations

% State Space description of uncontrolled bicycle
A = [-M0 \ C1 * v, -M0 \ (K0 + K2 * v^2); I, O];
B = [M0 \ [I, Hfw]; zeros(2, 3)];
C = eye(4);
D = zeros(4, 3);

% Combine A,B,C and D matrices into a state space object.
sys = ss(A, B, C, D);

end
