
function fig = IRFplot(np, mod)


h = np.h(1:np.m-2^11, :);
h_raw = np.h_raw(1:np.m-2^11, :);


tau = np.t(1:np.m-2^11) - np.t(1);
np.ylim = 5 / 3 * max(abs(np.h))' * ([-1, 1] + 1 / 5);

ya = {0.005, 0.01};

figure(200);
clf;
subplot(2, 1, 1);
title(['Finite Impulse Response for v=', num2str(mod.v, 2), ' m/s']);
hold on;
box off;
plot(tau, zeros(size(tau)), 'k:');
hold on;
box off;
hh(1) = plot(tau, h_raw(:, 1), 'b:');
hh(2) = plot(tau, h(:, 1), 'Color', 0.3*[0, 0, 1]);
legend(hh, 'h_\phi(\tau) raw', 'h_\phi(\tau) filtered');
xlim([tau(1), tau(end)]);
ylim(ya{1}*([-1, 1] + 1 / 3));
ylabel('IRF (rad/N)');
subplot(2, 1, 2);
plot(tau, zeros(size(tau)), 'k:');
hold on;
box off;
hh(1) = plot(tau, h_raw(:, 2), 'b:');
hh(2) = plot(tau, h(:, 2), 'Color', 0.3*[0, 0, 1]);

legend(hh, 'h_\delta(\tau) raw', 'h_\delta(\tau) filtered');
xlim([tau(1), tau(end)]);
ylim(ya{2}*([-1, 1] + 1 / 3));
ylabel('IRF (rad/N)');
xlabel('\tau (s)');


fig.hf = gcf;
