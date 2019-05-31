function en = lqrError(X, np, bike_m, dat)


Q=diag([X(1),X(2),X(3),X(4)]);
R=X(5);

a=1.;
[K,~,~]=lqr(bike_m.A,bike_m.B(:,2),Q,R);
output = lsim(bike_m.A-bike_m.B(:,2)*K,bike_m.B(:,3),bike_m.C,bike_m.D(:,3),a*dat.w,dat.t);

e = (output(:,3:4) - np.y);
en = ((sum(e.^2)) * 1 / np.N);
en=mean(en);
end