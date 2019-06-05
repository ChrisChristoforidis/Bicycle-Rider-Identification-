

in.pullforce=[dat.t',dat.w];
in.leantorque=[dat.t',zeros(dat.N,1)];
paramNameValStruct.StopTime= num2str(dat.t(end));
out= sim('state_fb_model_v2',paramNameValStruct);
%%


Q=diag([x_est(1),x_est(2),x_est(3),x_est(4),x_est(5),x_est(6)]);
R=x_est(7);

omegac = 2 * pi * 2.17; %2*pi*2.17;
s=tf('s');
Gnm = omegac^2 / (s^2 + 2 * sqrt(1/2) * omegac * s + omegac^2);
Gnm_ss=[];
[Gnm_ss.A,Gnm_ss.B,Gnm_ss.C,Gnm_ss.D] = ssdata(Gnm);
G_total.A=[Gnm_ss.A zeros(2,4);bike_m.B(:,2)*Gnm_ss.C bike_m.A];
G_total.B=[Gnm_ss.B;zeros(4,1)];
G_total.C=eye(6);
G_total.D=zeros(6,1);
G_total=ss(G_total.A,G_total.B,G_total.C,G_total.D);


[K,~,~]=lqr(G_total.A,G_total.B,Q,R);

output = lsim(G_total.A-G_total.B*K,[zeros(2,1); bike_m.B(:,3)],G_total.C,zeros(6,1),dat.w,dat.t);
out.roll_angle=output(:,end-1);
out.steer_angle=output(:,end);
out.SteerTorque=Gnm_ss.C*output(:,1:2).';
%%
figure(1223)
subplot(411)
plot(dat.t,dat.w);
ylabel("Lateral Force (N)");
legend("Measurement")
subplot(412)
plot(np.t,np.y(:,1));
ylabel("Roll Angle (Nm)");
hold on;
plot(out.tout,out.roll_angle);
legend("Non Parametric Model","Parametric Model")
subplot(413)
plot(np.t,np.y(:,2));
hold on;
plot(out.tout,out.steer_angle);
ylabel("Steer Angle (Nm)");
legend("Non Parametric Model","Parametric Model")
subplot(414)
plot(dat.t,dat.Tdelta);
hold on;
plot(out.tout,out.SteerTorque);
xlabel("Time (s)");
ylabel("Torque (Nm)");
legend("Measurement","Parametric Model")
