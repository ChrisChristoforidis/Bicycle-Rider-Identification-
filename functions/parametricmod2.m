
function [mod] = parametricmod2(npm, dat)

mod = struct('G', [], 'K', [], 'X', [], 'X0', [], 'z', [], 'y', [], 'C', []);


% mod.X0=[1.913174040654339e+02,16.941362159082970,1.350657591082278e+02,...
%     1.042079753727401e+02,53.240117796284046,27.308455419287228,...
%     14.400122990642188,-11.448913576856896];
mod.X0 = [71.186165140432620, 0.823393013408428, ...
  55.839413098998726, 37.203011117786076, 35.779390438899850, ...
  9.755311351232784, 3.524498497608346, -3.558349047280512];

mod.X0 = [48.506024581670450, -24.451034691470340, 59.099520789616236, 44.940566858446566, 32.515384990387020, -12.439528933296913, 4.189287968006891, -5.138894015576026];
j = 1;
bike = delftbike(dat.v);
bike_plant = bike(3:4, 2:3);
bike_plant.InputName = {'Steer Torque'; 'Lateral Force'};
bike_plant.OutputName = {'Roll Angle'; 'Steer Angle'};


bike_s = tf(bike_plant);

G11_num = bike_s.Numerator{1, 1};
G11_den = bike_s.Denominator{1, 1};
mod.G11 = tf(G11_num, G11_den);

G21_num = bike_s.Numerator{2, 1};
G21_den = bike_s.Denominator{2, 1};
mod.G21 = tf(G21_num, G21_den);

G12_num = bike_s.Numerator{1, 2};
G12_den = bike_s.Denominator{1, 2};
mod.G12 = tf(G12_num, G12_den);

G22_num = bike_s.Numerator{2, 2};
G22_den = bike_s.Denominator{2, 2};
mod.G22 = tf(G22_num, G22_den);


X0 = mod(j).X0;
X0n = ones(size(X0));
theta0 = X0(logical(X0)); %#ok<*AGROW>
theta0n = X0n(logical(X0)); %#ok<*AGROW>
e0 = norm(errorfunc2(theta0n, X0, 1, npm, j, mod(j), dat));
% Optimize model using the LSQNONLIN algorithm if flag is set to 1
[thetan, resnorm, en, exitflag, output, ~, Jn] = ...
  lsqnonlin(@(thetan)errorfunc2(thetan, X0, e0, npm, j, mod(j), dat), ...
  theta0n);



e = en .* e0;
N = length(e);
J = Jn ./ repmat(theta0, size(Jn, 1), 1) .* e0;
Xn = zeros(size(X0));
Xn(logical(X0)) = thetan;
X = Xn .* X0;
theta = thetan .* theta0;

% Optimization output
mod(j).covP = abs(1/N*(e' * e)*inv(J'*J)); %#ok<MINV>
mod(j).covPn = full(mod(j).covP) ./ (theta' * theta);
mod(j).sem = full(sqrt(diag(inv(J'*J))*sum(e.^2)/N));
mod(j).resnorm = resnorm;
mod(j).exitflag = exitflag;
mod(j).output = output;
mod(j).sel = logical(X0);

% Model output
mod = riderfunc2(X, tf('s'), mod(j));

delta_mod = lsim(mod.y(2), dat.w, dat.t);
delta = npm.y(:, 2);

mod.vaf = vaf(delta, delta_mod);
end


function en = errorfunc2(thetan, X0, e0, npm, j, mod, dat)
Xn = zeros(size(X0));
Xn(logical(X0)) = thetan;
X = Xn .* X0; % Renormalizing
mod = riderfunc2(X, tf('s'), mod); % X,s,i,mod
delta_mod = lsim(mod.y(2), dat.w, dat.t);
delta = npm.y(:, 2);
e = 1 / npm.N * (delta - delta_mod);
en = e / e0;
enn = (sum(en.^2));
%     ennn=(sum(e.^2));
end