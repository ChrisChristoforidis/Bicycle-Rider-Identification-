function dat=Roll_Observer(raw)
v=raw.v;%m/s 
%Reduction of samples to the true sampling frequency of IMU 
a=1;
OmegaZ_B=raw.GyroZ;%rad/s
OmegaZ_B=OmegaZ_B(1:a:end);
OmegaY_B=raw.GyroY;%rad/s
OmegaY_B=OmegaY_B(1:a:end);
OmegaX_B=raw.GyroX;%rad/s
OmegaX_B=OmegaX_B(1:a:end);
time=raw.time(1:a:end);
dt=time(2)-time(1);%new timestep
g=9.81;%m/s2
%Estimation of angle for low roll angles
phi_d=atan(OmegaZ_B*v/g);
%Estimation of roll angle for high roll angles
%   signZ=sign(OmegaZ_B);
%   phi_w=(signZ+eps).*asin(OmegaY_B./(sqrt(OmegaY_B.^2+OmegaZ_B.^2)+eps));%Gives NaN
%values
phi_w=atan(OmegaY_B./(OmegaZ_B+eps));%equivalent equation with the above
%idd=find(isnan(phi_w));%test if there are any NaN values in the array
%Correction of NaN values by averaging adjacant elements

% phi_w(idd(:))=(phi_w(idd(1)-1)+phi_w(idd(end)+1))/2;
% 
% sum(isnan(OmegaY_B./sqrt(OmegaY_B.^2+OmegaZ_B.^2)))



N=length(OmegaX_B);
a=0.05;% Constant value which can be used to adjust the behavior of the weighting

% Plant Discription
F=[1 -dt;0 1];
H=[1 0];
G=[dt; 0];
% Process And Measurement Noise
Qp=[5E-07 0 ; 0 1E-08];
Qs=0.1;

%Initialization of some cells and matrices

Pk=cell(N,1); %covariance matrix showing uncertainty in model output
K=cell(N,1); %kalman gains
X=cell(N,1); %State containing the roll angle and the bias 
ok=zeros(N,1); %Roll Angle  based on the absolute 'measurements'
yk=zeros(N-1,1);%Mismatch  between the expected 'sensor readings' and the model output
Sk=zeros(N-1,1);%Uncertainty in the system state projected via the sensor function
U=OmegaX_B;% input of the system is the roll rate
Ig=[1 0; 0 1];

%Intial Conditions 

Pk{1}=[0 0 ;0 0];%zero covariance matrix means we put 100% in the model output at the start
X{1}=[0 ;0]; % zero roll angle at the start and zero bias (no integration error)

 


%Estimation of Roll Angle purely based on the absolute 'measurements'
phi_m=zeros(N,1);
phi_m(1)=0;
for jj=1:N-1
     W=exp(-phi_m(jj)^2/a);
    phi_m(jj+1)=W*phi_d(jj+1)+(1-W)*phi_w(jj+1);

end

% Combined State Estimation with Kalman Filter implementation 
for jj=1:N-1
    %Prediction Step
    X{jj+1}=F*X{jj}+G*U(jj);
    Pk{jj+1}=F*Pk{jj}*F.'+Qp;
        W=exp(-X{jj}(1)^2/a);
        ok(jj+1)=W*phi_d(jj+1)+(1-W)*phi_w(jj+1);
         %KALMAN GAIN
    yk(jj)=ok(jj+1)-H*X{jj+1};
    Sk(jj)=H*Pk{jj+1}*H.'+Qs;
    K{jj+1}=Pk{jj+1}*H.'/Sk(jj);
     %UPDATE STEP
    X{jj+1}=X{jj+1}+K{jj+1}*yk(jj);
    Pk{jj+1}=(Ig-K{jj+1}*H)*Pk{jj+1};
end

phi=zeros(N,1);
for jj=1:N
phi(jj)=X{jj}(1);
end

bias=zeros(N,1);
for jj=1:N
bias(jj)=X{jj}(2);
end

KALMAN=zeros(N-1,2);
for jj=2:N
KALMAN(jj,:)=K{jj}(1:2);
end

COV=zeros(N-1,4);
for jj=2:N
COV(jj,1:2)=Pk{jj}(1:2);
COV(jj,3:4)=Pk{jj}(3:4);

end


dat=raw;
dat.RollAngle=phi;
dat.KALMAN=KALMAN;
dat.COV=COV;
dat.bias=bias;
dat.phi_all=[phi_d,phi_w,phi_m];
dat.Omega=[OmegaX_B,OmegaY_B,OmegaZ_B];

dat.time=time;
dat.N=length(dat.time);
end
