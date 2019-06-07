function fig =plotSimData(out,np,dat)
% Fucntion to plot the simulated data and compare them to the 
% non-parametric model ouput and the measured signals
% 
% Inputs 
% 
%     out : simulation output.Contains all the states of the bicycle.
%     np  : non-parametric model.Contains the steer angle and roll angle outputs.
%     dat : all measured signals.
%out.tout=dat.t;

out.tout=dat.t;
figure();
clf;
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
plot(out.tout,out.steer_torque);
xlabel("Time (s)");
ylabel("Torque (Nm)");
legend("Measurement","Parametric Model")

fig.hf=gcf;

end