function back= cutData(dat,n,m)
back=dat;
back.accel=dat.accel(n:m,:);
back.Tdelta=dat.Tdelta(n:m);
back.Rates=dat.Rates(n:m,:);

back.y=dat.y(n:m,:);
back.N=length(back.y);
back.w=dat.w(n:m);
back.t=dat.t(n:m);
end
