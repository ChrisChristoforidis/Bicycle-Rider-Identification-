mod.X0 = [1.913174040654339e+02, 16.941362159082970, 1.350657591082278e+02, ...
  1.042079753727401e+02, 53.240117796284046, 27.308455419287228, ...
  14.400122990642188, -11.448913576856896];

mod.X0 = [2.518310001167243e+02, -9.083259500033586, 1.818061609725173e+02, 2.132850258363073e+02, 1.446621740520228e+02, 15.328523132078674, 40.675286797319290, -23.814945329093728]
mod.X0 = x_est;
K1 = [mod.X0(4), mod.X0(3), mod.X0(1), mod.X0(2)];
K2 = [mod.X0(8), mod.X0(7), mod.X0(5), mod.X0(6)];


[simOut] = costfunc(mod.X0, dat);

function [simOut] = costfunc(X, dat)
roll_Gains = X(1:4);
steer_Gains = X(6:8);
open_system('rider_model_ver3');
set_param('rider_model_ver3/Rider/C1*Gnm', 'Numerator', 'K1');
set_param('rider_model_ver3/Rider/C2*Gnm', 'Numerator', 'K2');
input = [dat.t.', dat.w];
set_param('rider_model_ver3/Lateral Force', 'VariableName', 'input');
set_param('rider_model_ver3', 'StopTime', '6')
simOut = sim('rider_model_ver3', 'ReturnWorkspaceOutputs', 'on');
end