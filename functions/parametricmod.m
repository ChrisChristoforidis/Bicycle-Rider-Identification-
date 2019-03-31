function [mod] = parametricmod(npm, dat, x_est)

mod = struct('G', [], 'K', [], 'X', [], 'X0', [], 'z', [], 'y', [], 'C', []);

% mod.X0=[1.913174040654339e+02,16.941362159082970,1.350657591082278e+02,...
%     1.042079753727401e+02,53.240117796284046,27.308455419287228,...
%     14.400122990642188,-11.448913576856896];
mod.X0 = [1.993903697198388e+02, -3.978802214726249, ...
  1.195526041918253e+02, 1.640523127865040e+02, ...
  75.817662132379840, 10.138605147448544, 31.931184428289900, -20.199383776802783];

mod.X0 = x_est;
% For each model structure
j = 1;

% Initial parameters based on optimal control theory:
bike = delftbike(dat.v); % Bicycle model from Davis


% Bike
mod(j).G.yu = bike(3:4, 2);
mod(j).G.yw = bike(3:4, 3);
mod(j).G.zu = -bike(3, 2);
mod(j).G.zw = -bike(3, 3);

% Normalizing variables using initial parameter conditions.
X0 = mod(j).X0;
X0n = ones(size(X0));
theta0 = X0(logical(X0)); %#ok<*AGROW>
theta0n = X0n(logical(X0)); %#ok<*AGROW>
e0 = norm(errorfunc(theta0n, X0, 1, npm, j, mod(j), dat));
% Optimize model using the LSQNONLIN algorithm if flag is set to 1
[thetan, resnorm, en, exitflag, output, ~, Jn] = ...
  lsqnonlin(@(thetan)errorfunc(thetan, X0, e0, npm, j, mod(j), dat), ...
  theta0n);


% Unnormalizing output
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
mod(j).i = j;
mod(j) = riderfunc(X, tf('s'), mod(j));

delta_mod = lsim(mod(j).y(2), dat.w, dat.t);
delta = npm.y(:, 2);

mod(j).vaf = vaf(delta, delta_mod);


end


% Error definition
function en = errorfunc(thetan, X0, e0, npm, j, mod, dat)
Xn = zeros(size(X0));
Xn(logical(X0)) = thetan;
X = Xn .* X0; % Renormalizing
mod = riderfunc(X, tf('s'), mod); % X,s,i,mod
delta_mod = lsim(mod.y(2), dat.w, dat.t);
delta = npm.y(:, 2);
e = 1 / npm.N * (delta - delta_mod);
en = e / e0;
enn = (sum(en.^2));
%     ennn=(sum(e.^2));
end
