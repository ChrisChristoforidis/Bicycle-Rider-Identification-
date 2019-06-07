function [Md]=delayStateSpace(M,delay,K)
% Delay arbitrary state space system by discrete delay line 
% Changes the observation matrix, such that only delayed states are
% observable. Changes A, B, C and D accordingly.
%
% Inputs:
%   - M:        State Space Model to be delayed.
%
%   - delay:    Intended delay as in number of samples. 
%
% Outputs:
%   - Md:       Structure containing the new matrices to generate a the delayed model.
%               These are Md.a, Md.b, Md.c and Md.d for the extended versions
%               of the A, B, C and D matrices.
%               Matlab is case sensitive! Use exactly the same naming!
A = M.A;
B = M.B;
C = M.C;
D = M.D;
d = delay/M.Ts;

% lengths
U = size(B,2);  % num of cols
N = size(A,2);  % same
O = size(C,1);  % num of rows

% create delayed matrices
Md.A = [A,                  zeros(N,N*(d-1)),   zeros(N);...
        eye(N),             zeros(N,N*(d-1)),   zeros(N);...
        zeros(N*(d-1),N),   eye(N*(d-1)),       zeros(N*(d-1),N)];

Md.B = [B; zeros(N*d,U)];
Md.K = [K 0 0 zeros(1,N*d)];
Md.C = [zeros(O,N*d), C];

Md.D = D;
% -------------------------------------------------
% Taken from Joern Diedrichsen j.diedrichsen@bangor.ac.uk
% changed by Alexander Kuck a.kuck@utwente.nl
