K=[-19.067896194094440,3.507207707751709,-3.422831023102377,0.694606464802936];
out=modelSim(K,bike,dat);
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);
%Gp.Ts=0.001;
% Gpd=set(Gp,'InputDelay',1.000);
% %  Gp.InputDelay=[1.00;1.00;1.0];
% Gpd=delayStateSpace(Gp,0.025,K);
% out2 = lsim(Gpd.A,Gpd.B,eye(length(Gpd.A)),zeros(length(Gpd.A),3),[zeros(dat.N,1) out.Input.' dat.w],dat.t);
Gps=ss(Gp.A-Gp.B(:,2)*[K 0 0],Gp.B(:,3),Gp.C,Gp.D(:,1));
Gps=set(Gps,'InputDelay',0.025);
out1=lsim(Gps,dat.w,dat.t);
% out2=lsim(Gp,[dat.w],dat.t);

plot(out1(:,3));hold on
ylim([-0.1 0.1])


%% modelSimlink

open_system('state_fb_model_v4');
in.pullforce = [dat.t',dat.w];
in.leantorque = [dat.t',zeros(dat.N,1)];
in.a=[dat.t',out.Input.'];
paramNameValStruct.StopTime= num2str(dat.t(end));
set_param('state_fb_model_v4/Delay','DelayLength',num2str(100));
output= sim('state_fb_model_v4',paramNameValStruct);
plot(out.roll_angle)
hold on
plot(output.roll_angle)
ylim([-0.5 0.5])