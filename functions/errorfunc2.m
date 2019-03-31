function en = errorfunc2(X, np, mod, dat)

mod = riderfunc2(X, tf('s'), mod); % X,s,i,mod
delta_mod = lsim(mod.y(2), dat.w, dat.t);
delta = np.y(:, 2);
e = (delta - delta_mod);

en = (sum(e.^2)) * 1 / np.N;
end
