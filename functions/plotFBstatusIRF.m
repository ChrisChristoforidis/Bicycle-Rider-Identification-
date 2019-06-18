function plotFBstatusIRF(fb_np,nofb_np,fb_runs,nofb_runs)

figure()
for i=1:4
subplot(2,4,i)
plot(fb_np(i).tau(fb_np(i).tau>=0),fb_np(i).h(:,1));
hold on
plot(nofb_np(i).tau(nofb_np(i).tau>=0),nofb_np(i).h(:,1),'--');
if i==1
ylabel('Roll Angle IRF (rad/N)')
end
legend("FB ON at "+ num2str(fb_runs(i).v)+" m/s","FB OFF at "+ num2str(nofb_runs(i).v)+" m/s")
end
for i=1:4
subplot(2,4,4+i)
plot(fb_np(i).tau(fb_np(i).tau>=0),fb_np(i).h(:,2));
hold on
plot(nofb_np(i).tau(nofb_np(i).tau>=0),nofb_np(i).h(:,2),'--');
if i==1
ylabel('Steer Angle IRF (rad/N)')
end
legend("FB ON at "+ num2str(fb_runs(i).v)+" m/s","FB OFF at "+ num2str(nofb_runs(i).v)+" m/s")
end


end