close all; clear; clc;

%% 1-DOF base robotic arm
syms t th0(t) th1(t) th2(t) L1 L2 l1 l2 m0 m1 m2 I1 I2 g

x0 = th0;
y0 = 0;
x1 = th0 + l1*cos(th1);
y1 = l1*sin(th1);
x2 = th0 + L1*cos(th1) + l2*cos(th1+th2);
y2 = L1*sin(th1) + l2*sin(th1+th2);

dth1 = diff(th1,t);
dth2 = diff(th2,t);
dx0 = diff(x0,t);
dx1 = diff(x1,t);
dy1 = diff(y1,t);
dx2 = diff(x2,t);
dy2 = diff(y2,t);

KE0 = (1/2)*m0*dx0^2;
KE1 = (1/2)*m1*(dx1^2+dy1^2) + (1/2)*I1*dth1^2;
KE2 = (1/2)*m2*(dx2^2+dy2^2) + (1/2)*I2*dth2^2;

PE0 = 0;
PE1 = m1*g*l1*sin(th1);
PE2 = m2*g*(L1*sin(th1)+l2*sin(th1+th2));

KEtot = KE0 + KE1 + KE2;
PEtot = PE0 + PE1 + PE2;

Lag = KEtot - PEtot;

dLdth0 = diff(Lag,th0);
dLddx0 = diff(Lag,dx0);

dLdth1 = diff(Lag,th1);
dLddth1 = diff(Lag,dth1);

dLdth2 = diff(Lag,th2);
dLddth2 = diff(Lag,dth2);

u = diff(dLddx0,t) - dLdth0;
tau1 = diff(dLddth1,t) - dLdth1;
tau2 = diff(dLddth2,t) - dLdth2;

u = simplify(u);
tau1 = simplify(tau1);
tau2 = simplify(tau2);

disp(u)
disp(tau1)
disp(tau2)

%% 3-DOF base robotic arm
syms t xv(t) yv(t) alv(t) th1(t) th2(t) b1 b2 L1 L2 l1 l2 m0 m1 m2 I0 I1 I2 g

x0 = xv;
y0 = yv;
x1 = xv + b1/2*cos(alv) + l1*cos(alv+th1);
y1 = yv + b1/2*sin(alv) + l1*sin(alv+th1);
x2 = xv + b1/2*cos(alv) + L1*cos(alv+th1) + l2*cos(alv+th1+th2);
y2 = yv + b1/2*sin(alv) + L1*sin(alv+th1) + l2*sin(alv+th1+th2);

dalv = diff(alv,t);
dth1 = diff(th1,t);
dth2 = diff(th2,t);
dx0 = diff(x0,t);
dy0 = diff(y0,t);
dx1 = diff(x1,t);
dy1 = diff(y1,t);
dx2 = diff(x2,t);
dy2 = diff(y2,t);

KE0 = (1/2)*m0*(dx0^2+dy0^2) + (1/2)*I0*dalv^2;
KE1 = (1/2)*m1*(dx1^2+dy1^2) + (1/2)*I1*dth1^2;
KE2 = (1/2)*m2*(dx2^2+dy2^2) + (1/2)*I2*dth2^2;

PE0 = m0*g*y0;
PE1 = m1*g*y1;
PE2 = m2*g*y2;

KEtot = KE0 + KE1 + KE2;
PEtot = PE0 + PE1 + PE2;

Lag = KEtot - PEtot;

dLdalv = diff(Lag,alv);
dLddalv = diff(Lag,dalv);

dLdx0 = diff(Lag,x0);
dLddx0 = diff(Lag,dx0);

dLdy0 = diff(Lag,y0);
dLddy0 = diff(Lag,dy0);

dLdth1 = diff(Lag,th1);
dLddth1 = diff(Lag,dth1);

dLdth2 = diff(Lag,th2);
dLddth2 = diff(Lag,dth2);

ux = diff(dLddx0,t) - dLdx0;
uy = diff(dLddy0,t) - dLdy0;
tau0 = diff(dLddalv,t) - dLdalv;
tau1 = diff(dLddth1,t) - dLdth1;
tau2 = diff(dLddth2,t) - dLdth2;

ux = simplify(ux);
uy = simplify(uy);
tau0 = simplify(tau0);
tau1 = simplify(tau1);
tau2 = simplify(tau2);

disp(ux)
disp(uy)
disp(tau0)
disp(tau1)
disp(tau2)

%% simplify 
syms x xd xdd y yd ydd av avd avdd th_1 th_1d th_1dd th_2 th_2d th_2dd
oldvars = [xv(t), diff(xv(t),t), diff(xv(t),t,t), yv(t), diff(yv(t),t), diff(yv(t),t,t), ...
    alv(t), diff(alv(t),t), diff(alv(t),t,t), th1(t), diff(th1(t),t), diff(th1(t),t,t), ...
    th2(t), diff(th2(t),t), diff(th2(t),t,t)];
newvars = [x, xd, xdd, y, yd, ydd, av, avd, avdd, th_1, th_1d, th_1dd, th_2, th_2d, th_2dd];

ux = subs(ux,flip(oldvars),flip(newvars));
uy = subs(uy,flip(oldvars),flip(newvars));
tau0 = subs(tau0,flip(oldvars),flip(newvars));
tau1 = subs(tau1,flip(oldvars),flip(newvars));
tau2 = subs(tau2,flip(oldvars),flip(newvars));

ux = expand(ux,"ArithmeticOnly",true);
uy = expand(uy,"ArithmeticOnly",true);
tau0 = expand(tau0,"ArithmeticOnly",true);
tau1 = expand(tau1,"ArithmeticOnly",true);
tau2 = expand(tau2,"ArithmeticOnly",true);

disp("----------------expanded equations-------------------------")
disp(ux)
disp(uy)
disp(tau0)
disp(tau1)
disp(tau2)
