function [mod] = riderfunc(X, s, mod)


% Declarations/limitations
omegac = 2 * pi * 2.17; %2*pi*2.17;
Gnm = omegac^2 / (s^2 + 2 * sqrt(1/2) * omegac * s + omegac^2);

% Time delay:
delay = 1;
%     if i>6
%             z = -0.025*s;
%             delay = (1+1/2*z+1/12*z^2)/(1-1/2*z+1/12*z^2);
%     end

% Gain model
mod.X = X;
pid = [1; 1 / s; s; s^2];
mod.C = mod.X(1:8) * [pid, zeros(4, 1); zeros(4, 1), pid];
mod.K = mod.C * Gnm * delay;


% Calculate closed loop system responses
me = []; %#ok<NASGU>
mod.y = mod.G.yw + mod.G.yu * ((eye(1) - mod.K * mod.G.yu) \ mod.K * mod.G.yw);
%     [a,b]=ss2tf(mod.y.A,mod.y.B,mod.y.C,mod.y.D);
%     mod.G.phiu2=tf(a(1,:),b);
%     mod.G.deltau2=tf(a(2,:),b);
try mod.y = minreal(mod.y);
catch me;
end %#ok<NASGU>
mod.z = -mod.y(1);
