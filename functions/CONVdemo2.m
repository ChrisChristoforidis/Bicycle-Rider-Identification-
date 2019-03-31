function fig = CONVdemo2(np, mod, i, varargin)

if ~isempty(varargin)
  mod.w = varargin{1};
end
legend = {'h_{\phi}(\tau)', 'h_{\delta}(\tau)'; 'h_{\phi}(\tau) * w(t-\tau)', 'h_{\delta}(\tau) * w(t-\tau)'; '\phi(t)', '\delta(t)'};
% default variables
dt = 1 / mod.Fs; % sample time

M = sum(np.tau >= 0); % length of IRF
tau = np.tau;


m1 = -3 * (-2^9 + 2^8);
t = [(-tau(end):dt:-dt), mod.t, (np.t(end) + dt:dt:np.t(end) + dt * (M - 1))]; %extended time vecotor by 1 IRF length
ht = [zeros(m1, 1); np.h(1:M, i)]; % IRF

% input
w = [zeros(M-1, 1); mod.w; zeros(M-1, 1)];
a = 100; % # of samples skipped by each animation step
jj = 0;
figure(100);
clf;
for loop = 0:a:mod.N
  jj = jj + 1;
  
  subplot(411)
  %w(t)
  plot(t, w, 'LineWidth', 2), box off
  xlim([0, np.t(end)])
  ylabel('w(t)', 'FontSize', 12)
  
  subplot(412)
  % h(tau)
  plot(tau, ht, 'LineWidth', 2), box off
  set(gca, 'XDir', 'Reverse', 'XTick', [0, max(tau)])
  tmp = [-np.t(end), 0] + (loop - 1) * dt;
  xlim(tmp)
  ylim([min(ht), max(ht)])
  ylabel(legend{1, i}, 'FontSize', 12)
  
  
  subplot(413)
  % h(tau) * w(t-tau)
  u_h = w(1+loop:M+m1+loop);
  z_h = u_h .* flipud(ht);
  t_h = t(1+loop:M+m1+loop);
  fill(t_h, z_h, 'r', 'EdgeColor', 'r'), box off
  xlim([0, np.t(end)])
  if isempty(varargin)
    ylim([-1, 1])
  end
  ylabel(legend{2, i}, 'FontSize', 12)
  
  subplot(414)
  % y(t)
  y_h = sum(z_h) * dt;
  y_hh(jj) = y_h;
  plot(t(M+loop), y_h, 'g.'), box off, hold on
  xlim([0, np.t(end)])
  if isempty(varargin)
    ylim([min(np.y(:, i)), max(np.y(:, i))])
  end
  ylabel(legend{3, i}, 'FontSize', 12)
  xlabel('time (s)', 'FontSize', 14)
  pause(0.01)
  drawnow
end
if isempty(varargin)
  pause(1)
  y = np.y(:, i);
  figure(100)
  plot(np.t, y, 'g')
else
  pause(1)
  y = y_hh(:);
  figure(100)
  plot((0:dt * a:np.t(end)), y, 'g')
  
end

fig.hf = gcf;
