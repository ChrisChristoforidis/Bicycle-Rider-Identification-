function en = statefbError(X,np,bike_m, dat)
% s=tf('s');
addpath('simulink');
omegac = 2 * pi * 2.17; %2*pi*2.17;

Numerator=[omegac^2];
Denominator=[1 2*sqrt(1/2)*omegac omegac^2];

% Gnm = omegac^2 / (s^2 + 2 * sqrt(1/2) * omegac * s + omegac^2);
% Gnm_ss=[]
% [Gnm_ss.A,Gnm_ss.B,Gnm_ss.C,Gnm_ss.D] = ssdata(Gnm*X);

% Q=diag([X(1),X(2),X(3),X(4)]);
% R=X(5);
% 
% [K,~,~]=lqr(bike_m.A,bike_m.B(:,2),Q,R);

open_system('state_fb_model_v2');
in.pullforce=[dat.t',dat.w];
in.leantorque=[dat.t',zeros(dat.N,1)];

%set the parameters of the whipple bicycle model in the simulink model
set_param('state_fb_model_v2/State-Space','A',mat2str(bike_m.A));
set_param('state_fb_model_v2/State-Space','B',mat2str(bike_m.B));
set_param('state_fb_model_v2/State-Space','C',mat2str(bike_m.C));
set_param('state_fb_model_v2/State-Space','D',mat2str(bike_m.D));
%set the parameters of the rider model in the simulink model
set_param('state_fb_model_v2/Gain','Gain',mat2str(X));
set_param('state_fb_model_v2/Muscle','Numerator',mat2str(Numerator));
set_param('state_fb_model_v2/Muscle','Denominator',mat2str(Denominator));


paramNameValStruct.StopTime= num2str(dat.t(end));
try 
  out= sim('state_fb_model_v2',paramNameValStruct);
catch 
  out.roll_angle=ones(length(dat.N),1)*inf;
  out.steer_angle=ones(length(dat.N),1)*inf;
end

output=[out.roll_angle,out.steer_angle];

e = (output - np.y);
en = ((sum(e.^2)) * 1 / np.N);
en=en(1);
end