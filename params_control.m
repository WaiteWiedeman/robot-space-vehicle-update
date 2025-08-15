function params = params_control()    
    params = struct();
    % PID gains for each input (Kp, Ki, Kd)
    % controller = load("best_controller.mat");
    params.PID1 = [8.851181783816979e+02,3.945087347088166e+02,2.316946873141927e+03]; 
    params.PID2 = [8.843094432354505e+02,2.761455605376626e+02,2.473957858177560e+03];
    params.PID3 = [1.287580675287294e+03,4.809849376769460e+02,7.044922121377617e+02];
    params.PID4 = [9.282292312668094e+02,2.907060535067795e+02,9.335886909923696e+02];
    params.PID5 = [7.999941717716889e+02,3.105602543817336e+02,9.335886909923696e+02];
    params.dt_control = 0.001; % time step difference in which new control action is calculated
    params.Pf = 0.1;
    params.Flim = 200;
    params.Tlim = 50;
    params.refx = 1; % center of reference trajectory in x
    params.refy = 1; % center of reference trajectory in y
    params.xrange = 4; % width of reference point range
    params.yrange = 4; % height of reference point range
    params.a = 0.5; % target object horizontal dimension
    params.b = 0.25; % vertical dimension
    params.phi = 0; % angle of target point
    params.noise = 1;
    params.sigma = 5e-2;
    params.refrad = 0.5; % radius of reference trajectory 
    params.friction = 1; 
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "random"; % random, interval, origin
    params.numPoints = 1000;
    params.interval = 1e-3;
    params.solver = "nonstiff"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstiff"
end