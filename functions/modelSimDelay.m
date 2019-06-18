function out = modelSimDelay(K,bike,dat)
% Simulates the response of the measured input with the full closed loop model.
% Inputs 
%     K   :   The gains of the controller
%     bike: The EOM of the whipple model
%     dat :  The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque


% Get combined plant model (bicycle+neuromuscular dyanmics)



% Process
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);
Gp.Ts=0.001;
Gp_delayed=delayStateSpace(Gp,0.025,K);
Gpd=ss(Gp_delayed.A,Gp_delayed.B(:,2:end),Gp_delayed.C,Gp_delayed.D(:,2:3));
Gpd.y='yd';
Gpd.u={'a';'w'};
Gpd.Ts=0.001;
% Prediction model
omegac= 2 * pi * 2.17;
Gp=plantModel(bike,omegac);
Gp.Ts=0.001;
Gp_delayed=delayStateSpace(Gp,0.025,K);
temp=Gp_delayed.C(:,1:6);
temp2=Gp_delayed.C;
temp2(:,1:6)=temp2(:,end-5:end);
temp2(:,end-5:end)=temp;
Gmd=ss(Gp_delayed.A,Gp_delayed.B(:,2),[temp2;Gp_delayed.C],[Gp_delayed.D(:,2);Gp_delayed.D(:,2)]);
Gpd.y='yp';
a=Gpd.y;
Gpd.y='ypd';
b=Gpd.y;
Gpd.y='yd';
Gmd.y={a{:,1} b{:,1}};
Gmd.u={'a'};
Gmd.Ts=0.001;

% Overall plant
S1 = sumblk('ym = yp + dy');
S2 = sumblk('dy = yd - ypd');
Plant = connect(Gp,Gpd,S1,S2,{'a';'w'},'ym');


%Simulation of closed loop system
output = lsim(Gp.A-Gp.B(:,2)*[K 0 0],Gp.B(:,3),eye(6),zeros(6,1),dat.w,dat.t);
% Output assignment
out.steer_torque = output(:,5);
out.steer_angle = output(:,4);
out.roll_angle = output(:,3);
out.roll_rate = output(:,1);
out.steer_rate = output(:,2);


end