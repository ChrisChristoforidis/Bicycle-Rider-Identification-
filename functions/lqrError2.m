function en = lqrError2(X,np,bike_m, dat)

Q=diag([X(1),X(2),X(3),X(4),X(5),X(6)]);
R=X(7);

omegac = 2 * pi * 2.17; %2*pi*2.17;
Gnm_ss.A=[0 1; -omegac^2 -2*sqrt(1/2)*omegac];
Gnm_ss.B=[0;omegac^2];
Gnm_ss.C=[1 0];
Gnm_ss.D=0;

G_total.A=[Gnm_ss.A zeros(2,4);bike_m.B(:,2)*Gnm_ss.C bike_m.A];
G_total.B=[Gnm_ss.B;zeros(4,1)];
G_total.C=eye(6);
G_total.D=zeros(6,1);
G_total=ss(G_total.A,G_total.B,G_total.C,G_total.D);


[K,~,~]=lqr(G_total.A,G_total.B,Q,R);

output = lsim(G_total.A-G_total.B*K,[zeros(2,1); bike_m.B(:,3)],G_total.C,zeros(6,1),dat.w,dat.t);

e = (output(:,5:6) - np.y);
en = ((sum(e.^2)) * 1 / np.N);
en=mean(en);

end