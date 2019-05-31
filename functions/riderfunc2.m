function [paraMOD] = riderfunc2(X, s, mod)

paraMOD.K = X;

G22 = mod.G22;
G21 = mod.G21;
G12 = mod.G12;
G11 = mod.G11;

pidd = [1; 1 / s; s; s^2];
K1 = paraMOD.K(1:4);
K2 = paraMOD.K(5:8);
C1 = K1 * pidd;
C2 = K2 * pidd;
omegac = 2 * pi * 2.17;
Gnm = omegac^2 / (s^2 + 2 * sqrt(1/2) * omegac * s + omegac^2); %Neuromuscular dynamics
Gd = 1; %Time delay

Hy1w = (G12 + Gnm * Gd * C2 * G22 * G11 - Gnm * Gd * C2 * G21 * G12) / (1 - Gnm * Gd * C2 * G21 - Gnm * C1 * G11);
Hy2w = (G22 + Gnm * Gd * C1 * G12 * G21 - Gnm * Gd * C1 * G22 * G11) / (1 - Gnm * Gd * C2 * G21 - Gnm * C1 * G11);
% [SSy2w.A,SSy2w.B,SSy2w.C,SSy2w.D]=tf2ss(Hy2w.Numerator{1,1},Hy2w.Denominator{1,1});
% [SSy1w.A,SSy1w.B,SSy1w.C,SSy1w.D]=tf2ss(Hy1w.Numerator{1,1},Hy1w.Denominator{1,1});
% SSy1w=ss(SSy1w.A,SSy1w.B,SSy1w.C,SSy1w.D);
% SSy2w=ss(SSy2w.A,SSy2w.B,SSy2w.C,SSy2w.D);
paraMOD.y = [Hy1w; Hy2w];
% paraMOD.y=[SSy1w;SSy2w];

paraMOD.y.InputName = {'Lateral Force'};
paraMOD.y.OutputName = {'Roll Angle'; 'Steer Angle'};
paraMOD.y = minreal(paraMOD.y, 1e-3/2);
   
paraMOD.z = -Hy1w;
