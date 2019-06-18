
clear;
addpath(genpath('functions'), genpath('Measurements'),genpath('simulink'));
close all;
%%
load('C:\dev\bicycle-rider-control-identification\Measurements\all_data_steer.mat')
load('C:\dev\bicycle-rider-control-identification\Measurements\all_data_rope.mat')

load('C:\dev\bicycle-rider-control-identification\Measurements\all_data.mat')
load('C:\dev\bicycle-rider-control-identification\Measurements\olddata.mat')
%%

for i=1:length(alldata)
dat=alldata(i); 
np = nonparaID(dat);
fb_np(i)=np;
end

for i=1:length(nofb_results.data)
dat=nofb_results.data(i); 
np = nonparaID(dat);
nofb_np(i)=np;
end
%%
fb_results.data=fb_runs;
fb_results.black_box=fb_np;

nofb_results.data=nofb_runs;
nofb_results.black_box=nofb_np;

clearvars -except nofb_results fb_results
%save('C:\dev\bicycle-rider-control-identification\Measurements\all_data_rope.mat');

%%
i=1

dat=nofb_runs(i); 

figure('units','normalized','outerposition',[0 0 1 1])
plot(dat.t,dat.y)
ylim([-0.3 0.3])
yyaxis right
plot(dat.t,dat.w)
ylim([-250 250]);

%%

plotSpeedIRF(fb_results.black_box,fb_results.data)
plotSpeedIRF(nofb_results.black_box,nofb_results.data)

plotFBstatusIRF(fb_results.black_box,nofb_results.black_box,fb_results.data,nofb_results.data)



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
 
for i=1:4
 np=fb_results.black_box(i);
 dat=fb_results.data(i);
 bike = delftbike(dat.v,whipple); % Bicycle model from delft
 [K, fval, ~, ~] = fmincon(@(X)statefbError2(X,np,bike, dat), X0,[], [], [], [], lb, ub, [],options);
 output =modelSim(K,bike,dat);
 output.K=K;
 fb_results.final_model(i)= output;
 
end
%%
for i=1:4
output= fb_results.final_model(i) ;
np=fb_results.black_box(i);
dat=fb_results.data(i);
plotSimData(output,np,dat)
end
