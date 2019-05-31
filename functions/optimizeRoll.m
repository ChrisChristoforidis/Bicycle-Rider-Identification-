function en = optimizeRoll(X,dat,whipple)
M0 = whipple.M0;
C1 = whipple.C1;
K0 = whipple.K0;
Hfw = whipple.Hfw;
K2 = whipple.K2;

gamma_roll = [dat.accel(:,2),dat.v*dat.Rates(:,2),9.82*dat.y(:,2)];
YY_roll=Hfw(1)*dat.w(:)-M0(1,1)*dat.accel(:,1)-C1(1,1)*dat.v*dat.Rates(:,1)-K0(1,1)*dat.y(:,1)-K2(1,1)*dat.v^2*dat.y(:,1)-K2(1,2)*dat.v^2*dat.y(:,2);

e= gamma_roll*X.'-YY_roll;


en = (sum(e.^2)) * 1 / dat.N;

end
