s=tf('s');

omegac = 2 * pi * 2.17; %2*pi*2.17;
Gnm = omegac^2 / (s^2 + 2 * sqrt(1/2) * omegac * s + omegac^2);

pid = [1; 1 / s; s; s^2];
delay=1;
C = x_est * [pid, zeros(4, 1); zeros(4, 1), pid];
K = C * Gnm * delay;


u=dat.y;
time=dat.t;

Tdelta_model= lsim(K,u,time);
figure()
plot(Tdelta_model);

yyaxis right
plot(dat.Tdelta);


bike = delftbike(dat.v,whipple); % Bicycle model from delft

u = [dat.w,zeros(dat.N,1),Tdelta_model];
y_model=lsim(bike,u,dat.t);
figure()
plot(dat.t,y_model(:,1));
hold on;
plot(dat.t,dat.y(:,2))
ylim([-2 2])
legend("Model","Measument");