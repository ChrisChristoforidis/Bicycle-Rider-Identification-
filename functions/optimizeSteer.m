function en = optimizeSteer(X,dat,whipple)
M0 = whipple.M0;
C1 = whipple.C1;
K0 = whipple.K0;
Hfw = whipple.Hfw;
K2 = whipple.K2;


GAMMA=[dat.accel(:,2) ,dat.v*dat.Rates(:,1) ,dat.v*dat.Rates(:,2),dat.v^2*dat.y(:,2),-dat.w(:)]; 
YY=dat.Tdelta-M0(2,1)*dat.accel(:,1)-K0(2,2)*dat.y(:,2)-K2(2,1)*dat.v^2*dat.y(:,1) -9.82*dat.y(:,1)*K0(2,1); 

e= GAMMA*X.'-YY;


en = (sum(e.^2)) * 1 / dat.N;

end