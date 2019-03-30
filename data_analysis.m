
%% DATA ANALYSIS 

%CHRISTOS CHRISTOFORIDIS 
%20/11/2018 
clear;addpath('functions');
close all;
%%
load('C:\Users\chris\Desktop\DATA_ANALYSIS\Measurements\Test_23_11_2018.mat');

raw.Fs=1000;%hz
dt=1/raw.Fs;
raw.count=data(:,2);
raw.vel=data(:,10);
raw.N=length(raw.count);
raw.time=(0:raw.N-1)*dt;
raw.w=data(:,20);
raw.w=raw.w-median(raw.w);


idd=find(raw.w<500);


fc = 10;%hz (cutoff frequency)
Wn = pi*fc/(2*raw.Fs);
[c,d]=butter(4,Wn);
raw.w(idd)=filtfilt(c, d, raw.w(idd));
ii=0;
raw.w=raw.w-median(raw.w);
TF = islocalmax(raw.w,'MinProminence',50);
idd=find(TF);
raw.ww=zeros(raw.N,1);
for jj=1:length(idd)
    for ii=-300:400
    raw.ww(idd(jj)+ii)=raw.w(idd(jj)+ii);
    end
end
raw.w=raw.ww;
for jj=1:length(raw.w)
    if raw.w(jj)<0
        raw.w(jj)=0;
    end
end

%%
figure(1)
pause(0.00001);
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
plot(raw.time,raw.vel);grid on
yyaxis right
plot(raw.time,raw.w);
xlabel('Time (s)');
ylabel('Bike Velocity (m/s)');
title('Choose the time points between which the analysis is going to be performed ');
[x,~] = ginput(2);
close(1)
i1=round(x(1)/dt);
i2=round(x(2)/dt);



raw.time=raw.time(i1:i2);
raw.time=raw.time-min(raw.time);
raw.SteerAngle=data(i1:i2,5);
raw.w=raw.w(i1:i2);
raw.Tmotor=data(i1:i2,8);%Nm

%Filtering of Motor Input and SteerAngle
raw.Tmotor=filtfilt(c, d, raw.Tmotor);
raw.SteerAngle=filtfilt(c, d, raw.SteerAngle);
raw.SteerAngle=raw.SteerAngle-median(raw.SteerAngle);
%IMU Gyro data
raw.GyroX=-data(i1:i2,17);
raw.GyroY=-data(i1:i2,16);
raw.GyroZ=-data(i1:i2,18);
%Mean  forward speed
v=data(i1:i2,10);
raw.v=mean(v);

%%

dat=Roll_Observer(raw);

time=dat.time;
phi=dat.RollAngle;
phi_m=dat.phi_all(:,3);
phi_d=dat.phi_all(:,1);
phi_w=dat.phi_all(:,2);



%% TORQUE ESTIMATION


y=dat.SteerAngle.';
[Trider,v,dv,ddv,~]=TorqueEstimation(y,dat.Tmotor.');

if max(ddv)>40
figure(2)
frame_h = get(handle(gcf),'JavaFrame');
pause(0.00001);
set(frame_h,'Maximized',1);
plot(dat.time,ddv);grid on
xlabel('Index Counter');
ylabel('Steer Acceleration (rad/s^2)');
title('Cut the signal short at a  point where the interpolated acceleration is corrupted');
[x,~] = ginput(1);
close(2)
b1=round(x(1)/dt);
else 
    b1=length(ddv);
end

%FINAL NOMINATION oF THIS RUNS DATA

mod.Tdelta=Trider(1:b1).';
mod.y=[dat.RollAngle(1:b1),dat.SteerAngle(1:b1)];
mod.t=dat.time(1:b1);
mod.N=length(mod.Tdelta);
mod.Fs=dat.Fs;
mod.w=dat.w(1:b1);
mod.Rates=[dat.Omega(1:b1,1),dv(1:b1).'];
mod.v=dat.v;
v=v.';



%% PLOTS 


figure (3)
plot(time,phi_m,'k');hold on
plot(time,phi_d,'b');
plot(time,phi_w,'g:');
plot(time,phi,'r')
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('\phi_{m}','\phi_{d}','\phi_{w}','Observer');

figure(4)
plot(time(1:end),phi(1:end)*180/pi);hold on
plot(time(1:end),zeros(length(time(1:end)),1))
ylabel('Roll Angle (deg)');
yyaxis right
plot(time(1:end),dat.w(1:end))
ylabel('Lateral Force (N)');

xlabel('Time (s)');

figure(5)
subplot(311)
plot(mod.t,mod.y(:,2));hold on
plot(mod.t,v(1:b1),'--');
ylabel('Steer Angle (rad)')
legend('Orginal', 'Spline Interpolation');
subplot(312)
plot(mod.t,mod.Rates(:,2))
ylabel('Steer Rate (rad/s)')

subplot(313)
plot(mod.t,ddv(1:b1))
ylabel('Steer Acceleration (rad/s^2)')

xlabel('Time (s)');

  figure(6)
  plot(mod.t,mod.Tdelta);
  xlabel('Time(s)');
  ylabel('Torque(Nm)');
  yyaxis right
plot(time(1:b1),dat.w(1:b1))
ylabel('Lateral Force (N)');



raw=dat;
dat=mod;

clearvars -except raw dat




%% WHIPPLE MODEL FOR DELFT BIKE

% % Bike_whip = delftbike(mod.v);
% Bike_whip=Bike_whip(1,2); %Steer Torque/Roll Angle  
% 
%  [num_whip,dem_whip]=ss2tf(Bike_whip.A,Bike_whip.B,Bike_whip.C,Bike_whip.D);


%% NON PARAMETRIC ID
tic
np=nonparaID(dat);
toc

% %%
% fig = IRFplot(np,mod);
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
% fig2 = CONVdemo(np,mod,1);



%% genetic
   mod = struct('G',[],'K',[],'X',[],'X0',[],'z',[],'y',[],'C',[]);
  


   bike = delftbike(dat.v); % Bicycle model from Davis
   options = gaoptimset;                   % Load default options
   options = gaoptimset(options,...
    'populationsize', 80,...            % Number of individuals in population
    'elitecount', 8,...                 % Number of elites (elites move to the next generation without modification)
    'crossoverfraction', 0.85,...        % Fraction of population that is created by crossover [0..1]. Set too low: inbreeding; set too high: no convergence
    'popinitrange', [-25; 50]); 
            % Bike
            mod.G.yu =  bike(3:4,2);
            mod.G.yw =  bike(3:4,3);
            mod.G.zu = -bike(3,2);
            mod.G.zw = -bike(3,3);

            [x_est, fval, reason, output] = ga(@(X)errorfunc(X,np,mod,dat), 8,[],[],[],[],[],[],[],options);
   a=1;       
            modd=riderfunc2(x_est,tf('s'),mod);
 delta_mod_MINE = lsim(modd.y(1),dat.w*a,dat.t);
figure(3000)
plot(dat.t,delta_mod_MINE);hold on

plot(dat.t,np.y(:,1))
ylim([-1 1]);
FIT=vaf(np.y(:,1),delta_mod_MINE);
 %% gradient
[modd2] = parametricmod(np,dat,x_est);
%             % Bike
%                bike = delftbike(dat.v); % Bicycle model from Davis
% 
%             mod.G.yu =  bike(3:4,2);
%             mod.G.yw =  bike(3:4,3);
%             mod.G.zu = -bike(3,2);
%             mod.G.zw = -bike(3,3);
% modd2=riderfunc(modd2.X,tf('s'),mod);
      
delta_mod2 = lsim(modd2.y(2),dat.w,dat.t);
figure(3000)
plot(dat.t,delta_mod2);hold on

plot(dat.t,np.y(:,2))
ylim([-1 1]);
%%

   mod = struct('G',[],'K',[],'X',[],'X0',[],'z',[],'y',[],'C',[]);
  


 
   options = optimoptions('ga');                   % Load default options
   options = optimoptions(options,...
    'populationsize', 80,...            % Number of individuals in population
    'elitecount', 8,...                 % Number of elites (elites move to the next generation without modification)
    'crossoverfraction', 0.8,...        % Fraction of population that is created by crossover [0..1]. Set too low: inbreeding; set too high: no convergence
    'popinitrange', [-25; 50]); 
            % Bike
           
bike = delftbike(dat.v);
bike_plant=bike(3:4,2:3);
bike_plant.InputName={'Steer Torque';'Lateral Force'};
bike_plant.OutputName={'Roll Angle';'Steer Angle'};


bike_s=tf(bike_plant);

G11_num=bike_s.Numerator{1,1};
G11_den=bike_s.Denominator{1,1};
mod.G11=tf(G11_num,G11_den);

G21_num=bike_s.Numerator{2,1};
G21_den=bike_s.Denominator{2,1};
mod.G21=tf(G21_num,G21_den);

G12_num=bike_s.Numerator{1,2};
G12_den=bike_s.Denominator{1,2};
mod.G12=tf(G12_num,G12_den);

G22_num=bike_s.Numerator{2,2};
G22_den=bike_s.Denominator{2,2};
mod.G22=tf(G22_num,G22_den);

          [x_est, fval, reason, output] = ga(@(X)errorfunc2(X,np,mod,dat), 8,[],[],[],[],[],[],[],options);
             %%

[mod] = parametricmod2(np,dat);
 modd=riderfunc2(mod.K,tf('s'),mod);


delta_mod_MINE = lsim(modd.y(1),dat.w,dat.t);
figure(3001)
plot(dat.t,delta_mod_MINE);hold on

plot(dat.t,np.y(:,2))
ylim([-1 1]);

delta_mod_MINE = lsim(modd.y(2),dat.w(end/2:end),dat.t(end/2:end));
figure(3000)
plot(dat.t(end/2:end),delta_mod_MINE);hold on

plot(dat.t(end/2:end),np.y(end/2:end,2))
ylim([-1 1]);

