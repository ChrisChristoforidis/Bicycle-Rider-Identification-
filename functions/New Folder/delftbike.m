function sys = delftbike(v)

% input v: forward velocity of the bicycle.
% output sys: state space model of the bicycle.


    % Coefficient matrices;

  load('JBike6MCK.mat','C1','M0','K2','K0')

    Hfw = [0.9; 0.014408]; % dfx/dTq

    % External parameter;
  

    O = zeros(2); I = eye(2); % Some easy notations

    % State Space description of uncontrolled bicycle
    A = [O I; -M0\(K0 + K2*v^2) -M0\C1*v];
    B = [zeros(2,3); M0\[I Hfw]];
    C = eye(4);
    D = zeros(4,3);

    % Combine A,B,C and D matrices into a state space object.
    sys = ss(A,B,C,D);

end
