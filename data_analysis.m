
%% DATA ANALYSIS

%CHRISTOS CHRISTOFORIDIS
%20/11/2018
clear;
addpath('functions', "Measurements");
close all;

%%
load('Test_23_11_2018.mat');

raw.Fs = 1000; %hz
dt = 1 / raw.Fs;
raw.count = data(:, 2);
raw.vel = data(:, 10);
raw.N = length(raw.count);
raw.time = (0:raw.N - 1) * dt;
raw.w = data(:, 20);
raw.w = raw.w - median(raw.w);


idd = find(raw.w < 500);


fc = 5; %hz (cutoff frequency)
Wn = pi * fc / (2 * raw.Fs);
[c, d] = butter(4, Wn);
raw.w(idd) = filtfilt(c, d, raw.w(idd));
ii = 0;
raw.w = raw.w - median(raw.w);
TF = islocalmax(raw.w, 'MinProminence', 100);
idd = find(TF);


raw.ww = zeros(raw.N, 1);
for jj = 1:length(idd)
  for ii = -300:400
    raw.ww(idd(jj)+ii) = raw.w(idd(jj)+ii);
  end
end
raw.w = raw.ww;
for jj = 1:length(raw.w)
  if raw.w(jj) < 0
    raw.w(jj) = 0;
  end
end

%%
figure(1)
pause(0.00001);
frame_h = get(handle(gcf), 'JavaFrame');
set(frame_h, 'Maximized', 1);
plot(raw.time, raw.vel);
grid on
yyaxis right
plot(raw.time, raw.w);
xlabel('Time (s)');
ylabel('Bike Velocity (m/s)');
title('Choose the time points between which the analysis is going to be performed ');
[x, ~] = ginput(2);
close(1)
i1 = round(x(1)/dt);
i2 = round(x(2)/dt);


raw.time = raw.time(i1:i2);
raw.time = raw.time - min(raw.time);
raw.SteerAngle = data(i1:i2, 5);
raw.w = raw.w(i1:i2);
raw.Tmotor = data(i1:i2, 8); %Nm

%Filtering of Motor Input and SteerAngle
raw.Tmotor = filtfilt(c, d, raw.Tmotor);
raw.SteerAngle = filtfilt(c, d, raw.SteerAngle);
raw.SteerAngle = raw.SteerAngle - median(raw.SteerAngle);
%IMU Gyro data
raw.GyroX = -data(i1:i2, 17);
raw.GyroY = -data(i1:i2, 16);
raw.GyroZ = -data(i1:i2, 18);
%Mean  forward speed
v = data(i1:i2, 10);
raw.v = mean(v);

%% Roll Estimation

dat = Roll_Observer(raw);


time = dat.time;
phi = dat.RollAngle;
phi_m = dat.phi_all(:, 3);
phi_d = dat.phi_all(:, 1);
phi_w = dat.phi_all(:, 2);

%% TORQUE ESTIMATION


y = dat.SteerAngle.';
[Trider, v, dv, ddv, ~] = TorqueEstimation(y, dat.Tmotor.',true);

if max(ddv) > 40
  figure(2)
  frame_h = get(handle(gcf), 'JavaFrame');
  pause(0.00001);
  set(frame_h, 'Maximized', 1);
  plot(dat.time, ddv);
  grid on
  xlabel('Index Counter');
  ylabel('Steer Acceleration (rad/s^2)');
  title('Cut the signal short at a  point where the interpolated acceleration is corrupted');
  [x, ~] = ginput(1);
  close(2)
  b1 = round(x(1)/dt);
else
  b1 = length(ddv);
end
dat.Omega(1:b1, 1) = filtfilt(c,d,dat.Omega(1:b1, 1));
RollAccel = (dat.Omega(2:b1, 1) - dat.Omega(1:b1-1, 1))*dat.Fs;
RollAccel(end+1)=RollAccel(end);
%FINAL NOMINATION oF THIS RUNS DATA
mod.accel = [RollAccel,ddv.'];
mod.Tdelta = Trider(1:b1).';
mod.y = [dat.RollAngle(1:b1), dat.SteerAngle(1:b1)];
mod.t = dat.time(1:b1);
mod.N = length(mod.Tdelta);
mod.Fs = dat.Fs;
mod.w = dat.w(1:b1);
mod.Rates = [dat.Omega(1:b1, 1), dv(1:b1).'];
mod.v = dat.v;
v = v.';

%% PLOTS
% 
% 
% figure(3)
% plot(time, phi_m, 'k');
% hold on
% plot(time, phi_d, 'b');
% plot(time, phi_w, 'g:');
% plot(time, phi, 'r')
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('\phi_{m}', '\phi_{d}', '\phi_{w}', 'Observer');
% 
% figure(4)
% plot(time(1:end), phi(1:end)*180/pi);
% hold on
% plot(time(1:end), zeros(length(time(1:end)), 1))
% ylabel('Roll Angle (deg)');
% yyaxis right
% plot(time(1:end), dat.w(1:end))
% ylabel('Lateral Force (N)');
% 
% xlabel('Time (s)');
% 
% figure(5)
% subplot(311)
% plot(mod.t, mod.y(:, 2));
% hold on
% plot(mod.t, v(1:b1), '--');
% ylabel('Steer Angle (rad)')
% legend('Orginal', 'Spline Interpolation');
% subplot(312)
% plot(mod.t, mod.Rates(:, 2))
% ylabel('Steer Rate (rad/s)')
% 
% subplot(313)
% plot(mod.t, ddv(1:b1))
% ylabel('Steer Acceleration (rad/s^2)')
% 
% xlabel('Time (s)');
% 
% figure(6)
% plot(mod.t, mod.Tdelta);
% xlabel('Time(s)');
% ylabel('Torque(Nm)');
% yyaxis right
% plot(time(1:b1), dat.w(1:b1))
% ylabel('Lateral Force (N)');


raw = dat;
dat = mod;

clearvars -except raw dat

%% NON PARAMETRIC ID
tic
np = nonparaID(dat);
toc

%%
%fig = IRFplot(np,np);
% figure(8)
% subplot(311)
% plot(mod.t,mod.y(:,2));
% ylabel('Angle (rad)');
% legend('y_{\delta}')
% subplot(312)
% plot(np.t,np.y(:,2));hold on
% ylabel('Angle (rad)');
% legend('g_{\delta}*w')
% subplot(313)
% plot(np.t,np.n(:,2));hold on
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('v_{\delta}')
% 
% 
% figure(9)
% subplot(311)
% plot(mod.t,mod.y(:,1));
% ylabel('Angle (rad)');
% legend('y_{\phi}')
% subplot(312)
% plot(np.t,np.y(:,1));hold on
% ylabel('Angle (rad)');
% legend('g_{\phi}*w')
% subplot(313)
% plot(np.t,np.n(:,1));hold on
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% legend('v_{\phi}')
%

%% Demonstration of convolution
% u=randn(mod.N,1)*30;
% fig2 = CONVdemo(np,dat,1);

%% Steer-by-wire Bicycle-Whipple model 
load('JBike6MCK.mat', 'C1', 'M0', 'K2', 'K0')

a = -fliplr(eye(2)) + eye(2);
M0 = a .* M0;
C1 = C1 .* a;
K0 = K0 .* a;
K2 = K2 .* a;
Hfw = [0.84; 0.014408]; % dfx/dTq

whipple.M0 = M0;
whipple.C1 = C1;
whipple.K0 = K0;
whipple.K2 = K2;
whipple.Hfw = Hfw;

bike = delftbike(dat.v,whipple); % Bicycle model from delft




%% LQR design 
roll=zeros(dat.N,1);
roll(1)=0;
for i=1:dat.N-1
  roll(i+1)=roll(i)+dat.Rates(i,1)/dat.Fs;
  
end

dat.roll=highpass(roll,0.05,dat.Fs);

dat.y(:,1)=dat.roll;
%% Gradient Descent

 Q=diag([1 1 1/max(dat.Rates(:,1))^2 , 1/max(dat.Rates(:,2))^2 ,1/max(dat.y(:,1))^2 ,1/max(dat.y(:,2))^2]);
 R=1/max(dat.Tdelta)^2;
 lb=[0;0;0;0;0;0;0];
 ub=[inf;inf;inf;inf;inf;inf;inf];
 options = optimset('TolFun', 1e-8);

[x_est, fval, ~, ~] = fmincon(@(X)statefbError(X,np,bike, dat), X0,[], [], [], [], lb, ub, [],options);

%% Genetic Algorithm
in.pullforce=[dat.t',dat.w];
in.leantorque=[dat.t',zeros(dat.N,1)];
 lb=[-250;-250;-250;-250];
 ub=[250;250;250;250];

options = optimoptions('ga', "PopulationSize", 80, ...
  'EliteCount', 4, 'CrossoverFraction', 0.85, ...
  'InitialPopulationRange', [-50; 50],...
  'InitialPopulationMatrix',[-27.8688060508863 3.83424352600571 3.24115313899635 -1.4528375845157]);

[x_est, fval, ~, ~] = ga(@(X)statefbError(X, np, bike, dat), 4,[], [], [], [], lb, ub, [],options);
%X0=[-47.634443333666490,5.591059100165730,1.336398615765447,-20.907426280657038];
%X0=[-27.8688060508863 3.83424352600571 3.24115313899635 -1.4528375845157]
%minimize steer tf
%K=[-19.067896194094440,3.507207707751709,-3.422831023102377,0.694606464802936]
% minimize roll tf

%% Results
Q = diag(x_est(1:end-1));
R = x_est(end);

[K,~,~]=lqr(bike.A,bike.B(:,2),Q,R);
output = lsim(bike.A-bike.B(:,2)*K,bike.B(:,3),bike.C,bike.D(:,3),dat.w,dat.t);
TorqueRider = -K*output';


figure(12)
subplot(611)
plot(np.t,np.y(:,1));
ylabel("Roll Angle (Nm)");
hold on;
plot(dat.t,output(:,3));
legend("Non Parametric Model","Parametric Model")
subplot(612)
plot(np.t,np.y(:,2));
hold on;
plot(dat.t,output(:,4));
ylabel("Steer Angle (Nm)");
legend("Non Parametric Model","Parametric Model")
subplot(613)
plot(dat.t,dat.Tdelta);
hold on;
plot(dat.t,TorqueRider);
xlabel("Time (s)");
ylabel("Torque (Nm)");
legend("Measurement","Parametric Model")
subplot(614)
plot(dat.t,dat.Rates(:,1));
hold on;
plot(dat.t,output(:,1));
xlabel("Time (s)");
ylabel("Roll rate (rad/s)");
legend("Measurement","Parametric Model")
subplot(615)
plot(dat.t,dat.Rates(:,2));
hold on;
plot(dat.t,output(:,2));
xlabel("Time (s)");
ylabel("Steer rate (rad/s)");
legend("Measurement","Parametric Model")
subplot(616)
plot(dat.t,dat.accel(:,2));
hold on;
plot(dat.t(1:end-1),(output(2:end,2)-output(1:end-1,2))*dat.Fs);
xlabel("Time (s)");
ylabel("Steer Acceleration (rad/s^2)");
legend("Measurement","Parametric Model")



%% genetic
mod = struct('G', [], 'K', [], 'X', [], 'X0', [], 'z', [], 'y', [], 'C', []);
X=[];

options = optimoptions('ga', "PopulationSize", 80, ...
  'EliteCount', 8, 'CrossoverFraction', 0.85, ...
  'InitialPopulationRange', [-25; 50]);


% Bike
mod.G.yu = bike(3:4, 2);
mod.G.yw = bike(3:4, 3);
mod.G.zu = -bike(3, 2);
mod.G.zw = -bike(3, 3);

[x_est, ~, ~, ~] = ga(@(X)errorfunc(X, np, mod, dat), 8, [], [], [], [], [], [], [], options);

%x_est= [50.386881794668426,-5.405141150415773,38.818136655572300,43.519327272608660,34.304308697388940,-2.100885573064807,8.291874129698888,-4.564757689320331]
%% Result


a = 1.15;
modd = riderfunc(x_est, tf('s'), mod);
output_pid(:,1) = lsim(modd.y(1), dat.w*a, dat.t);
output_pid(:,2) = lsim(modd.y(2), dat.w*a, dat.t);

s=tf('s');
omegac = 2 * pi * 2.17;
Gnm = omegac^2 / (s^2 + 2 * sqrt(1/2) * omegac * s + omegac^2);

mod.X = x_est;
pid = [1; 1 / s; s; s^2];
mod.C = mod.X(1:8) * [pid, zeros(4, 1); zeros(4, 1), pid];
mod.K = mod.C * Gnm ;

TorqueRider =lsim(mod.K,output_pid,dat.t);

figure(123)
subplot(411)
plot(dat.t,dat.w*a);
ylabel("Lateral Force (N)");

legend("Measurement")
subplot(412)
plot(np.t,np.y(:,1));
ylabel("Roll Angle (Nm)");
hold on;
plot(dat.t,output_pid(:,1));
legend("Non Parametric Model","Parametric Model")
subplot(413)
plot(np.t,np.y(:,2));
hold on;
plot(dat.t,output_pid(:,2));
ylabel("Steer Angle (Nm)");
legend("Non Parametric Model","Parametric Model")
subplot(414)
plot(dat.t,dat.Tdelta);
hold on;
plot(dat.t,TorqueRider);
xlabel("Time (s)");
ylabel("Torque (Nm)");
legend("Measurement","Parametric Model")


%% gradient
%[modd2] = parametricmod(np, dat, x_est);
%             % Bike
%                bike = delftbike(dat.v); % Bicycle model from Davis
%
%             mod.G.yu =  bike(3:4,2);
%             mod.G.yw =  bike(3:4,3);
%             mod.G.zu = -bike(3,2);
%             mod.G.zw = -bike(3,3);
% modd2=riderfunc(modd2.X,tf('s'),mod);
% 
% delta_mod2 = lsim(modd2.y(2), dat.w, dat.t);
% figure(3000)
% plot(dat.t, delta_mod2);
% hold on
% 
% plot(dat.t, np.y(:, 2))
% ylim([-1, 1]);

%%

mod = struct('G', [], 'K', [], 'X', [], 'X0', [], 'z', [], 'y', [], 'C', []);


options = optimoptions('ga'); % Load default options
options = optimoptions(options, ...
  'populationsize', 80, ... % Number of individuals in population
  'elitecount', 8, ... % Number of elites (elites move to the next generation without modification)
  'crossoverfraction', 0.8, ... % Fraction of population that is created by crossover [0..1]. Set too low: inbreeding; set too high: no convergence
  'popinitrange', [-25; 50]);


% Bike

bike = delftbike(dat.v);
bike_plant = bike(3:4, 2:3);
bike_plant.InputName = {'Steer Torque'; 'Lateral Force'};
bike_plant.OutputName = {'Roll Angle'; 'Steer Angle'};


bike_s = tf(bike_plant);

G11_num = bike_s.Numerator{1, 1};
G11_den = bike_s.Denominator{1, 1};
mod.G11 = tf(G11_num, G11_den);

G21_num = bike_s.Numerator{2, 1};
G21_den = bike_s.Denominator{2, 1};
mod.G21 = tf(G21_num, G21_den);

G12_num = bike_s.Numerator{1, 2};
G12_den = bike_s.Denominator{1, 2};
mod.G12 = tf(G12_num, G12_den);

G22_num = bike_s.Numerator{2, 2};
G22_den = bike_s.Denominator{2, 2};
mod.G22 = tf(G22_num, G22_den);

[x_est, fval, reason, output] = ga(@(X)errorfunc2(X, np, mod, dat), 8, [], [], [], [], [], [], [], options);

%%

[mod] = parametricmod2(np, dat);
modd = riderfunc2(mod.K, tf('s'), mod);


delta_mod_MINE = lsim(modd.y(1), dat.w, dat.t);
figure(3001)
plot(dat.t, delta_mod_MINE);
hold on

plot(dat.t, np.y(:, 2))
ylim([-1, 1]);

delta_mod_MINE = lsim(modd.y(2), dat.w(end/2:end), dat.t(end/2:end));
figure(3000)
plot(dat.t(end/2:end), delta_mod_MINE);
hold on

plot(dat.t(end/2:end), np.y(end/2:end, 2))
ylim([-1, 1]);
