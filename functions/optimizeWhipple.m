function en = optimizeWhipple(X,dat)
%X(1)=M0(1,2)
%X(2)=C1(1,2)
%X(3)=K0(1,2)


load('JBike6MCK.mat', 'C1', 'M0', 'K2', 'K0')

a = -fliplr(eye(2)) + eye(2);
M0 = a .* M0;
C1 = C1 .* a;
K0 = K0 .* a;
K2 = K2 .* a;
Hfw = [0.9; 0.014408]; % dfx/dTq
% Identification of whipple model parameters


gamma_roll = [dat.accel(:,2),dat.v*dat.Rates(:,2),9.82*dat.y(:,2)];
YY_roll=Hfw(1)*dat.w(:)-M0(1,1)*dat.accel(:,1)-C1(1,1)*dat.v*dat.Rates(:,1)-K0(1,1)*dat.y(:,1)-K2(1,1)*dat.v^2*dat.y(:,1)-K2(1,2)*dat.v^2*dat.y(:,2);

e= gamma_roll*X.'-YY_roll;


en = (sum(e.^2)) * 1 / dat.N;

end
