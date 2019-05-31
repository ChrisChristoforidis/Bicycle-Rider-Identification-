function [npm] = nonparaID(mod)

dt = 1 / mod.Fs;
t = mod.t;
npm.N = mod.N;
npm.m = 5 * 2^10;

% FIR model of input related noise:
fir = FIR(mod); % Finite impulse response
M = sum(fir.tau >= 0);
h = fir.h(fir.tau >= 0, :);
h_raw = fir.h_raw(fir.tau >= 0, :);


% Convolution of IRF with Measured Input to get model output
w = [zeros(M-1, 1); mod.w];
y = zeros(npm.N, 2);

for ii = 1:size(mod.y, 2)
  
  % h(tau) * u(t-tau)
  for loop = 0:npm.N - 1
    u_h = w(1+loop:M+loop);
    z_h = u_h .* flipud(h(:, ii));
    y(loop+1, ii) = sum(z_h) * dt;
  end
  
end


% Remnant
n = mod.y - y; % Remnant

% Non parametric response results:
npm.n = n;
npm.t = t;
npm.y = y;
npm.h = h;
npm.v = mod.v;
npm.tau = fir.tau;
npm.h_raw = h_raw;
