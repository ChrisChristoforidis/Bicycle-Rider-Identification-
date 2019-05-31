

in.pullforce=[dat.t',dat.w];
in.leantorque=[dat.t',zeros(dat.N,1)];
paramNameValStruct.StopTime= num2str(dat.t(end));
out= sim('state_fb_model_v2',paramNameValStruct);

%%
figure(123)
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
