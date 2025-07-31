function [Xd, Yd, Xdd, Ydd, Xv, Xvd, Yv, Yvd] = referenceTrajectory(t,ctrlParams,sysParams)
    Xd = ctrlParams.refx + ctrlParams.a*cos(ctrlParams.phi)*cos((2*pi/5)*t) - ...
        ctrlParams.b*sin(ctrlParams.phi)*sin((2*pi/5)*t); 

    Yd = ctrlParams.refy + ctrlParams.b*sin(ctrlParams.phi)*cos((2*pi/5)*t) + ...
        ctrlParams.a*cos(ctrlParams.phi)*sin((2*pi/5)*t); 

    Xdd = -(2*pi/5)*ctrlParams.a*cos(ctrlParams.phi)*sin((2*pi/5)*t) - ...
        (2*pi/5)*ctrlParams.b*sin(ctrlParams.phi)*cos((2*pi/5)*t);

    Ydd = -(2*pi/5)*ctrlParams.b*sin(ctrlParams.phi)*sin((2*pi/5)*t) + ...
        (2*pi/5)*ctrlParams.a*cos(ctrlParams.phi)*cos((2*pi/5)*t);
    
    l1 = sysParams.L1;
    l2 = sysParams.L2;
    b1 = sysParams.b1/2;
    Xv = Xd - b1 - (l1 + l2)/2;
    Yv = Yd - b1 - (l1 + l2)/2;
    Xvd = Xdd;
    Yvd = Ydd;
end
