function out = modelSimlink(K,bike_m,dat)
% Simulates the response of the measured input with the full closed loop
% model. (using simulink)
% Inputs 
%     K      :   The gains of the controller
%     bike_m :   The EOM of the whipple model
%     dat    :   The full measured dataset
% Outputs
%     out :  contains all 4 states of bicycle.(dphi,ddelta,phi,delta) +
%     Rider Torque

addpath('simulink');
omegac = 2 * pi * 2.17; 

open_system('state_fb_model_v2');
in.pullforce = [dat.t',dat.w];
in.leantorque = [dat.t',zeros(dat.N,1)];

%set the parameters of the whipple bicycle model in the simulink model
set_param('state_fb_model_v2/State-Space','A',mat2str(bike_m.A));
set_param('state_fb_model_v2/State-Space','B',mat2str(bike_m.B));
set_param('state_fb_model_v2/State-Space','C',mat2str(bike_m.C));
set_param('state_fb_model_v2/State-Space','D',mat2str(bike_m.D));
%set the parameters of the rider model in the simulink model
set_param('state_fb_model_v2/Gain','Gain',mat2str(K));
set_param('state_fb_model_v2/Muscle','Numerator',mat2str(omegac^2));
set_param('state_fb_model_v2/Muscle','Denominator',mat2str([1 2*sqrt(1/2)*omegac omegac^2]));


paramNameValStruct.StopTime= num2str(dat.t(end));
try 
  out= sim('state_fb_model_v2',paramNameValStruct);
catch 
  out.roll_angle=ones(length(dat.N),1)*inf;
  out.steer_angle=ones(length(dat.N),1)*inf;
end
out.steer_torque=out.SteerTorque;

end