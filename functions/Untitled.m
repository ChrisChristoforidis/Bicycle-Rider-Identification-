function [mod] = riderfunc2(X,s,mod)


    % Declarations/limitations
    omegac = 2*pi*2.17; %2*pi*2.17;
    Gnm = omegac^2/(s^2 + 2*sqrt(1/2)*omegac*s + omegac^2);

    % Time delay:
    delay = 1;
    %     if i>6
    %         z = -0.030*s;
    %         delay = (1+1/2*z+1/12*z^2)/(1-1/2*z+1/12*z^2);
    %     end

    % Gain model
    mod.X = X;
    pid = [1;1/s;s;s^2];
    mod.C =  mod.X(1:8)*[pid zeros(4,1); zeros(4,1) pid];
    mod.K =  mod.C*Gnm*delay;

    %     % Discretization
    %     Ts = 0.005;
    %     mod.K = c2d(mod.K,Ts);
    %     mod.G.yu = c2d(mod.G.yu,Ts);
    %     mod.G.yw = c2d(mod.G.yw,Ts);

    % Calculate closed loop system responses
    me = []; %#ok<NASGU>
%     [a,b]=ss2tf(mod.G.yw.A,mod.G.yw.B,mod.G.yw.C,mod.G.yw.D);
%     mod.G.phiw=tf(a(1,:),b);
%     mod.G.deltaw=tf(a(2,:),b);
%     [a,b]=ss2tf(mod.G.yu.A,mod.G.yu.B,mod.G.yu.C,mod.G.yu.D);
%     mod.G.phiu=tf(a(1,:),b);
%     mod.G.deltau=tf(a(2,:),b);
%     mod.Gphiw =  mod.G.phiw + mod.G.phiu*(inv(eye(1)-mod.K(1)*mod.G.phiu-mod.K(2)*mod.G.deltau)\(mod.K(1)*mod.G.phiw+mod.K(2)*mod.G.deltaw));
%     mod.Gdeltaw = mod.G.deltaw + mod.G.deltau*((eye(1)-mod.K(2)*mod.G.deltau-mod.K(1)*mod.G.phiu)\(mod.K(2)*mod.G.deltaw+mod.K(1)*mod.G.phiw));
    mod.y =  mod.G.yw + mod.G.yu*((eye(1)-mod.K*mod.G.yu)\mod.K*mod.G.yw);
%     [a,b]=ss2tf(mod.y.A,mod.y.B,mod.y.C,mod.y.D);
%     mod.G.phiu2=tf(a(1,:),b);
%     mod.G.deltau2=tf(a(2,:),b);
    try mod.y =  minreal(mod.y); catch me; end %#ok<NASGU>
    mod.z = -mod.y(1);
