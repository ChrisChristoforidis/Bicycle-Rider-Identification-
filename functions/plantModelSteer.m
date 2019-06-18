function mod = plantModelSteer(bike,omegac)

bike.B(:,3)=bike.B(:,2);

Gnm.A=[0 1; -omegac^2 -2*sqrt(1/2)*omegac];
Gnm.B=[0;omegac^2];
Gnm.C=[1 0];
Gnm.D=0;

Gnm=ss(Gnm.A,Gnm.B,Gnm.C,Gnm.D);
Gnm.u='a'; %neural input
Gnm.y='Tdelta';%rider steer torque

bike.u(1)=cellstr('Tlean'); %rider lean torque
bike.u(2)=cellstr('Tdelta'); 
bike.u(3)=cellstr('w'); %lateral force

bike.y='y';
input={'Tlean';'a';'w'};
mod=connect(bike,Gnm,input,'y');


end
