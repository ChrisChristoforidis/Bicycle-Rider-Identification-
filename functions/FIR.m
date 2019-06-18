function fir = FIR(dat)

% Settings
m1 = 2 * (-2^9 + 2^8); % tau_min samples
m2 = 3 * (2^9 + 2^8); % tau_max samples
wn = 0.05 / 5; % Nyquist normalized datter factor

% FIR finite impulse response function h(tau)
N = size(dat.y, 2);

% Timeshift vector tau
DeltaT = 1 ./ dat.Fs;
tau = DeltaT * (m1:m2)';
M = length(tau);

% FIR windowing
shift = 5 * (2^6);
win = zeros(size(tau));
win(tau > 0) = 1;
win(end-shift:end) = 0;
win((m2 + 1:end)-shift) = 1 / 2 * (1 + cos((0:-m1)*pi/(-m1)));

% datter parameters
[b, a] = butter(4, wn);

% For every output y
h = zeros(M, N);
h_raw = zeros(M, N);

tic
for i = 1:N
  h_raw(:, i) = leastSQUARE(dat.y(:, i), dat.w, m1, m2) / DeltaT;
  h(:, i) = filtfilt(b, a, h_raw(:, i));
  h(:, i) = h(:, i) .* win;
  
end
toc
% Output

fir.tau = tau;
fir.h_raw = h_raw;
fir.h = h;


end
