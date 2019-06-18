function out = modelSim(K,bike,dat)
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
if max(abs(dat.w)) < 20
  Gp.B(:,3)=[bike.B(:,2) ;0 ;0];
end
%Simulation of closed loop system
output = lsim(Gp.A-Gp.B(:,2)*[K 0 0],Gp.B(:,3),eye(6),zeros(6,1),dat.w,dat.t);
% Output assignment
out.steer_torque = output(:,5);
out.steer_angle = output(:,4);
out.roll_angle = output(:,3);
out.roll_rate = output(:,1);
out.steer_rate = output(:,2);
out.Input= -K*output(:,1:4).';

end