function dat =filterLatForce(raww,fc,ub)
raw=raww;

figure(1)
pause(0.00001);
frame_h = get(handle(gcf), 'JavaFrame');
set(frame_h, 'Maximized', 1);
plot(raw.time, raw.w);
xlabel('Time (s)');
ylabel('Lateral Force (N)');
[x, ~] = ginput(2);
close(1)
dt=0.001;
i1 = round(x(1)/dt);
i2 = round(x(2)/dt);

offset=median(raw.w(i1:i2));

raw.w=raw.w-offset;
dat=raww;

Wn = pi * fc / (2 * raw.Fs);
[c, d] = butter(4, Wn);


if (min(raww.w)<-100)
  
temp=find(raw.w>0);
raw.w(temp)=0;
raw.w = raw.w - median(raw.w);
TF = islocalmax(abs(raw.w), 'MinProminence', 100);
idd = find(TF);
idxx=find(abs(raw.w(TF))>100);
idd=idd(idxx);


raw.w = filtfilt(c, d, raw.w);

raw.ww = zeros(raw.N, 1);
for jj = 1:length(idd)
  for ii = -350:350
    raw.ww(idd(jj)+ii) = raw.w(idd(jj)+ii);
  end
end
raw.w = raw.ww;
for jj = 1:length(raw.w)
  if raw.w(jj) > 0
    raw.w(jj) = 0;
  end
end

dat.w2=raw.w;
else 
  dat.w2 = zeros(length(raw.w),1);
end


raw=raww;
raw.w=raw.w-median(raw.w);


temp=find(raw.w<0);
raw.w(temp)=0;
raw.w = raw.w - median(raw.w);
TF = islocalmax(abs(raw.w), 'MinProminence', 100);
idd = find(TF);
idxx=find(abs(raw.w(TF))>ub);
idd=idd(idxx);

raw.w = filtfilt(c, d, raw.w);
raw.ww = zeros(raw.N, 1);
for jj = 1:length(idd)
  for ii = -350:350
    raw.ww(idd(jj)+ii) = raw.w(idd(jj)+ii);
  end
end
raw.w = raw.ww;
for jj = 1:length(raw.w)
  if raw.w(jj) < 0
    raw.w(jj) = 0;
  end
end
dat.w1=raw.w;
dat.w=dat.w1+dat.w2;

end
