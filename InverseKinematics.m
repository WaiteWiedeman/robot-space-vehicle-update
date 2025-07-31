function [Alv,Th1,Th2,Alvd,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xv,Xvd,Yv,Yvd)
sysParams = params_system();

l1 = sysParams.L1;
l2 = sysParams.L2;
b1 = sysParams.b1;

phiE = atan2((Yd-Yv),(Xd-Xv)); %

Xw = Xd - l2*cos(phiE);
Yw = Yd - l2*sin(phiE);

phiW = atan2((Yw-Yv),(Xw-Xv));

Th1 = pi - acos((l1^2+l2^2-(Xw-Xv)^2-(Yw-Yv)^2)/2/l1/l2);

Alv = phiW - acos(((Xw-Xv)^2+(Yw-Yv)^2+l1^2-l2^2)/2/l1/sqrt((Xw-Xv)^2+(Yw-Yv)^2));

Th2 = phiE - Th1 - Alv;

Jac = [-b1/2*sin(Alv)-l1*sin(Alv+Th1)-l2*sin(Alv+Th1+Th2) -l1*sin(Alv+Th1)-l2*sin(Alv+Th1+Th2) -l2*sin(Alv+Th1+Th2)
        b1/2*cos(Alv)+l1*cos(Alv+Th1)+l2*cos(Alv+Th1+Th2)  l1*cos(Alv+Th1)+l2*cos(Alv+Th1+Th2)  l2*cos(Alv+Th1+Th2)];
Oms = pinv(Jac)*[Xdd - Xvd ; Ydd - Yvd];

Om1 = Oms(1);
Om2 = Oms(2);
Alvd = Oms(3);
