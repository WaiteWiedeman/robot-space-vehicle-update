function T_f = friction(th1d,th2d,sysParams)
T_f = zeros(2,1);

mu_v = sysParams.mu_v;

T_f(1) = -th1d*mu_v;
T_f(2) = -th2d*mu_v;