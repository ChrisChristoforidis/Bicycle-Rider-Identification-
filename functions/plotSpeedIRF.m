
function plotSpeedIRF(fb_np,fb_runs)

figure()
subplot(211)

plot(fb_np(1).tau(fb_np(2).tau>=0),fb_np(1).h(:,1));
hold on
plot(fb_np(2).tau(fb_np(2).tau>=0),fb_np(2).h(:,1));
plot(fb_np(3).tau(fb_np(2).tau>=0),fb_np(3).h(:,1));
plot(fb_np(4).tau(fb_np(2).tau>=0),fb_np(4).h(:,1));
ylabel('Roll Angle IRF')
if fb_runs(1).FB==true
title("FEEDBACK ON");
else 
title("FEEDBACK OFF");
end
  
legend( num2str(fb_runs(1).v),num2str(fb_runs(2).v),num2str(fb_runs(3).v),num2str(fb_runs(4).v))
subplot(212)
plot(fb_np(1).tau(fb_np(2).tau>=0),fb_np(1).h(:,2));
hold on
plot(fb_np(2).tau(fb_np(2).tau>=0),fb_np(2).h(:,2));
plot(fb_np(3).tau(fb_np(2).tau>=0),fb_np(3).h(:,2));
plot(fb_np(4).tau(fb_np(2).tau>=0),fb_np(4).h(:,2));
ylabel('Steer Angle IRF')
legend( num2str(fb_runs(1).v),num2str(fb_runs(2).v),num2str(fb_runs(3).v),num2str(fb_runs(4).v))
end