function [Trider, v, dv, ddv, u] = TorqueEstimation(y, Tmotor)
Ihb = 0.0960; %Inertia of handlebars (calculated from handlebari_inertia_motor_damping.m)
b = 0.2663; %MOTOR DAMPING  (calculated from handlebari_inertia_motor_damping.m)
dt = 0.001;
n = length(y);
t = (0:n) * dt;
u = linspace(0, t(end), n+1);
t = t(1:n);
u = u(1:end-1); %points that the spline functions are going to be calculated for.

[v, dv, ddv] = mysplinetx(t, y, u);
Trider = ddv * Ihb + Tmotor(1:end) + b * dv;

end