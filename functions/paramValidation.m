 %% BIKE MODEL VALIDATION
 g=9.81;
%whipple model parameters
   M0 = [ 133.31668525,    2.43885691;  2.43885691,   0.22419262];
   C1 = [   0.0       ,   44.65783277; -0.31500940,   1.46189246];
   K0 = [-116.73261635,   -2.48042260; -2.48042260,  -0.77434358];
   K2 = [   0.0       ,  104.85805076;  0.00000000,   2.29688720];
   H = [0.91; 0.014408];
   
   ACCEL=zeros(dat.N,2);
   ACCEL(1,:)=(dat.y(2,3)-dat.y(1,3))*dat.Fs;
   ACCEL(end,:)=(dat.y(end,3)-dat.y(end-1,3))*dat.Fs;
   for jj=2:dat.N-1
   ACCEL(jj,1)=(dat.y(jj+1,3)-dat.y(jj-1,3))*dat.Fs/2;
   ACCEL(jj,2)=(dat.y(jj+1,4)-dat.y(jj-1,4))*dat.Fs/2;
   end
%    figure
%      % plot(dat.t,ACCEL,'--');hold on
%    plot(dat.t,dat.y(:,3:4));
%     legend('roll acceleration','steer acceleration','roll rate', 'steer rate') 

 
 GAM_roll=[ACCEL(:,2),dat.v*dat.y(:,4),g*dat.y(:,2)];

 YY_roll=H(1)*dat.w(:)-M0(1,1)*ACCEL(:,1)-C1(1,1)*dat.v*dat.y(:,3)-K0(1,1)*g*dat.y(:,1)-K2(1,1)*dat.v^2*dat.y(:,1)-K2(1,2)*dat.v^2*dat.y(:,2);
 

PARAM_roll= (GAM_roll'*GAM_roll)\(GAM_roll'*YY_roll);
% error_roll= ((GAM_roll*PARAM_roll)-YY_roll);
% 
 %M0(1,2)=PARAM_roll(1);
 %M0(2,1)=PARAM_roll(1);
C1(1,2)=PARAM_roll(2);
% K0(1,2)=PARAM_roll(3);

GAM_steer=[ACCEL(:,2) ,dat.v*dat.y(:,3) ,dat.v*dat.y(:,4),g*dat.y(:,1),dat.v^2*dat.y(:,2),-dat.w(:)]; 
YY_steer=dat.Tdelta-M0(2,1)*ACCEL(:,1)-K0(2,2)*g*dat.y(:,2)-K2(2,1)*dat.v^2*dat.y(:,1); 
PARAM_steer= (GAM_steer'*GAM_steer)\(GAM_steer'*YY_steer);
% error_steer= (GAM_steer*PARAM_steer)-YY_steer;
% cov_steer=(error_steer'*error_steer)/(dat.N-length(PARAM_steer));
M0(2,2)=PARAM_steer(1);
C1(2,1)=PARAM_steer(2);
C1(2,2)=PARAM_steer(3);
K0(2,1)=PARAM_steer(4);
K0(2,1)=K0(1,2);
K2(2,2)=PARAM_steer(5);
H(2)=PARAM_steer(6);
% plot(error)
%% LEAST SQUARES UNITED
N=dat.N;
  M0 = [ 133.31668525,    2.43885691;  2.43885691,   0.22419262];
   C1 = [   0.0       ,   44.65783277; -0.31500940,   1.46189246];
   K0 = [-116.73261635,   -2.48042260; -2.48042260,  -0.77434358];
   K2 = [   0.0       ,  104.85805076;  0.00000000,   2.29688720];
   H = [0.91; 0.014408];
%steer equation
% GAM_steer=[ACCEL(:,1),zeros(N,1),g*dat.y(:,1),ACCEL(:,2),dat.v*dat.y(:,3) ,dat.v*dat.y(:,4),dat.v^2*dat.y(:,2),-dat.w(:)]; 
% YY_steer=dat.Tdelta-K0(2,2)*g*dat.y(:,2)-K2(2,1)*dat.v^2*dat.y(:,1); 

GAM_steer=[ACCEL(:,1),zeros(N,1),g*dat.y(:,1),ACCEL(:,2),dat.v*dat.y(:,3) ,dat.v*dat.y(:,4),dat.v^2*dat.y(:,2),-dat.w(:)]; 
YY_steer=dat.Tdelta-K0(2,2)*g*dat.y(:,2)-K2(2,1)*dat.v^2*dat.y(:,1); 

%roll equation 
% GAM_roll=[ACCEL(:,2),dat.v*dat.y(:,4),g*dat.y(:,2),zeros(N,1),zeros(N,1),zeros(N,1),zeros(N,1),zeros(N,1)];
% YY_roll=H(1)*dat.w(:)-M0(1,1)*ACCEL(:,1)-C1(1,1)*dat.v*dat.y(:,3)-K0(1,1)*g*dat.y(:,1)-K2(1,1)*dat.v^2*dat.y(:,1)-K2(1,2)*dat.v^2*dat.y(:,2);
  
GAM_roll=[ACCEL(:,2),dat.v*dat.y(:,4),g*dat.y(:,2),zeros(N,1),zeros(N,1),zeros(N,1),zeros(N,1),zeros(N,1)];
YY_roll=H(1)*dat.w(:)-M0(1,1)*ACCEL(:,1)-C1(1,1)*dat.v*dat.y(:,3)-K0(1,1)*g*dat.y(:,1)-K2(1,1)*dat.v^2*dat.y(:,1)-K2(1,2)*dat.v^2*dat.y(:,2);

%combined equations
 GAMA=[GAM_steer;GAM_roll];
 YY=[YY_steer;YY_roll];
 %LEAST SQUARES SOLUTION
 PARAM= (GAMA'*GAMA)\(GAMA'*YY);
 
M0(1,2)=PARAM(1);
 M0(2,1)=PARAM(1);
 C1(1,2)=PARAM(2);
 K0(1,2)=PARAM(3);
K0(2,1)=K0(1,2);
 M0(2,2)=PARAM(4);
 C1(2,1)=PARAM(5);
 C1(2,2)=PARAM(6);
 K2(2,2)=PARAM(7);
 H(2)=PARAM(8);

error=GAMA*PARAM-YY;
plot(error)
%% IDENTIFIED SYSTEM RESPONSE 
   O = zeros(2); I = eye(2); % Some easy notations
   v=dat.v;
    % State Space description of uncontrolled bicycle
    A = [-M0\C1*v -M0\(K0*g + K2*v^2); I O];
    B = [ M0\[I H]; zeros(2,3)];
    C = eye(4);
    D = zeros(4,3);

    % Combine A,B,C and D matrices into a state space object.
    Bike_id= ss(A,B,C,D);
    Bike_id=Bike_id(3,2);
    %y_id=lsim(Bike_id,[dat.Tdelta,dat.w],dat.t)    ;

%% WHIPPLE MODEL RESPONSE 
Bike_whip = davisbike(v);
Bike_whip=Bike_whip(3,2);   
% y_whipple=lsim(Bike_whip,[dat.Tdelta,dat.w],dat.t)    ;

 [num_whip,dem_whip]=ss2tf(Bike_whip.A,Bike_whip.B,Bike_whip.C,Bike_whip.D);
 [num_id,dem_id]=ss2tf(Bike_id.A,Bike_id.B,Bike_id.C,Bike_id.D);
 figure(1)
  bode(Bike_id);hold on
  bode(Bike_whip,'--')
  xlim([0.1 100])
  legend('ID','WHIPPLE');
% figure
% plot(dat.t(1:end-8500),y_whipple(1:end-8500,1));hold on
% plot(dat.t(1:end-8500),y_id(1:end-8500,1));
