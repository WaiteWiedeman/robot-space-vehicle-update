function x = addnoise(t, x, ctrlParams)
% rng(1)
sz = size(x);
sig = ctrlParams.sigma;
x = x + sig*randn(sz); 
% xm = normrnd(0,sig,sz);
% disp("addnoise")
% disp(xm)
% disp(xm-x)
% xm = x + sig.*ones(sz).*sin(2*pi/0.1*t);
