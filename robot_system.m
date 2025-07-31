function dxdt = robot_system(t, x, noise, sysParams, ctrlParams)
    % t1 = tic;
    [Xd, Yd, Xdd, Ydd, Xv, Xvd, Yv, Yvd] = referenceTrajectory(t, ctrlParams,sysParams);
    % tEnd1 = toc(t1);
    % t2 = tic;
    [Alv,Th1,Th2,Alvd,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xv,Xvd,Yv,Yvd);
    % tEnd2 = toc(t2);
    % t3 = tic;
    F = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, noise, ctrlParams);
    if ctrlParams.friction 
        T_f = friction(x(8),x(10),sysParams);
    else
        T_f = zeros(2,1);
    end
    % tEnd3 = toc(t3);
    % t4 = tic;
    dxdt = robot_xdot(x, F, T_f, sysParams);
    % tEnd4 = toc(t4);
    % fprintf("Reference trajectory eval time: %.4f s\n", tEnd1);
    % fprintf("inverse kin eval time: %.4f s\n", tEnd2);
    % fprintf("forces eval time: %.4f s\n", tEnd3);
    % fprintf("xdot eval time: %.4f s\n", tEnd4);
end