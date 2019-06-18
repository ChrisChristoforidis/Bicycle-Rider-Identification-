
clear;
addpath(genpath('functions'), genpath('Measurements'),genpath('simulink'));
close all;
%%
load('C:\dev\bicycle-rider-control-identification\Measurements\all_data_steer.mat')

%%
for i=1:length(alldata)
dat=alldata(i); 
np = nonparaID(dat);
steer_result(i).Data=dat;
steer_result(i).BackBox=np;

 end


%%
% i=2
% 
% dat=alldata(i); 
% 
% figure('units','normalized','outerposition',[0 0 1 1])
% plot(dat.t,dat.y)
% ylim([-0.3 0.3])
% yyaxis right
% plot(dat.t,dat.w)
% ylim([-15 15]);

%%
results.black_box= [ steer_result(1).BackBox steer_result(2).BackBox steer_result(3).BackBox steer_result(4).BackBox ];
results.data= [ steer_result(1).Data steer_result(2).Data steer_result(3).Data steer_result(4).Data ];

%%

plotSpeedIRF(results.black_box,results.data)
i=4
figure()
plot(results.black_box(i).y)
yyaxis right 
plot(results.data(i).w)
figure()
plot(results.data(i).y)
yyaxis right 
plot(results.data(i).w)
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



%%

 lb=[-250;-250;-250;-250];
 ub=[250;250;250;250];
 options = optimset('TolFun', 1e-8);
 X0=[-19.067896194094440,3.507207707751709,-3.422831023102377,0.694606464802936];
 
for i=1
 np=results.black_box(i);
 dat=results.data(i);
 bike = delftbike(dat.v,whipple); % Bicycle model from delft
 [K, fval, ~, ~] = fmincon(@(X)statefbError2(X,np,bike, dat), X0,[], [], [], [], lb, ub, [],options);
 output =modelSim(K,bike,dat);
 output.K=K;
 results.final_model(i)= output;
 
end
%%
for i=1
  
output= results.final_model(i) ;
np=results.black_box(i);
dat=results.data(i);
plotSimData(output,np,dat)
end
