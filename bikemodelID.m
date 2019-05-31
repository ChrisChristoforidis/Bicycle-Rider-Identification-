%% Bicycle Model Validation

%% Steer-by-wire Bicycle-Whipple model 
load('JBike6MCK.mat', 'C1', 'M0', 'K2', 'K0')

a = -fliplr(eye(2)) + eye(2);
M0 = a .* M0;
C1 = C1 .* a;
K0 = K0 .* a;
K2 = K2 .* a;
Hfw = [0.84; 0.014408]; % dfx/dTq

whipple.M0 = M0;
whipple.C1 = C1;
whipple.K0 = K0;
whipple.K2 = K2;
whipple.Hfw = Hfw;

bike = delftbike(dat.v,whipple); % Bicycle model from delft



%% Roll Equation

X=[];
X(1)=M0(1,2);
X(2)=C1(1,2);
X(3)=K0(1,2);
% Identification of whipple model parameters
lb=[-inf;-inf;-inf];
% lb=[];
ub=[inf;inf;inf];
% ub=[];
[x_est,fval, ~, ~] = fmincon(@(X)optimizeRoll(X,dat,whipple), X, [], [], [], [], lb, ub, []);

GAMMA = [dat.accel(:,2),dat.v*dat.Rates(:,2),9.82*dat.y(:,2)];
YY=Hfw(1)*dat.w(:)-M0(1,1)*dat.accel(:,1)-C1(1,1)*dat.v*dat.Rates(:,1)-K0(1,1)*dat.y(:,1)-K2(1,1)*dat.v^2*dat.y(:,1)-K2(1,2)*dat.v^2*dat.y(:,2);

e= GAMMA*x_est.'-YY;
en = (sum(e.^2)) * 1 / dat.N;
% PARAM_steer= (gamma_roll'*gamma_roll)\(gamma_roll'*YY_roll);

whipple.M0(1,2) = x_est(1);
whipple.M0(2,1) = whipple.M0(1,2);
whipple.C1(1,2) = x_est(2);
whipple.K0(1,2) = x_est(3);
whipple.K0(2,1) = whipple.K0(1,2);


%% Steer Equation
X=[];
X(1)=M0(2,2);
X(2)=C1(2,1);
X(3)=C1(2,2);
X(4)=K2(2,2);
X(5)=Hfw(2);

lb=[0;-10;0;0;0];
 lb=[];
ub=[inf;0;10;10;0.1];
 ub=[];
[x_est,fval, ~, ~] = fmincon(@(X)optimizeSteer(X,dat,whipple), X, [], [], [], [], lb, ub, []);

whipple.M0(2,2) = x_est(1);
whipple.C1(2,1) = x_est(2);
whipple.C1(2,2) = x_est(3);
whipple.K2(2,2) = x_est(4);
whipple.Hfw(2) = x_est(5);
%%
bike = delftbike(dat.v,whipple); % Bicycle model from delft

%%
% GAM_steer=[dat.accel(:,2) ,dat.v*dat.Rates(:,1) ,dat.v*dat.Rates(:,2),9.82*dat.y(:,1),dat.v^2*dat.y(:,2),-dat.w(:)]; 
% YY_steer=dat.Tdelta-M0(2,1)*dat.accel(:,1)-K0(2,2)*dat.y(:,2)-K2(2,1)*dat.v^2*dat.y(:,1); 
% PARAM_steer= (GAM_steer'*GAM_steer)\(GAM_steer'*YY_steer);
M0 = whipple.M0;
C1 = whipple.C1;
K0 = whipple.K0;
Hfw = whipple.Hfw;
K2 = whipple.K2;
GAM_steer=[dat.accel(:,2) ,dat.v*dat.Rates(:,1) ,dat.v*dat.Rates(:,2),dat.v^2*dat.y(:,2),-dat.w(:)]; 
YY_steer=dat.Tdelta-M0(2,1)*dat.accel(:,1)-K0(2,2)*dat.y(:,2)-K2(2,1)*dat.v^2*dat.y(:,1)- K0(1,2)*9.82*dat.y(:,1); 
PARAM_steer= (GAM_steer'*GAM_steer)\(GAM_steer'*YY_steer);
e= GAM_steer*PARAM_steer-YY_steer;
plot(YY_steer);hold on;plot(GAM_steer*x_est.');

en = (sum(e.^2)) * 1 / dat.N;

%%
u = [dat.Tdelta,dat.w];
y_model=lsim(bike.A,bike.B(:,2:3),u,dat.t);
figure()
plot(dat.t,y_model(:,3));
hold on;
plot(dat.t,dat.y(:,2))
ylim([-2 2])
legend("Model","Measument");

%%

N=length(RollAngle);
T=length(RollAngle)/200;
timeJ=linspace(0,T,N);
bike = davisbike(mean(ForwardSpeed)); % Bicycle model from delft
u = [zeros(length(RollAngle),1),SteerTorque,PullForce];
y_model=lsim(bike,u,timeJ);

figure()
plot(timeJ,y_model(:,4));
hold on;
plot(timeJ,SteerAngle)
ylim([-2 2])
legend("Model","Measument");
