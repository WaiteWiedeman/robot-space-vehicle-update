function params = params_control()    
    params = struct();
    % PID gains for each input (Kp, Ki, Kd)
    params.PID1 = []; 
    params.PID2 = [];
    params.PID3 = []; 
    params.PID4 = []; 
    params.PID5 = [];
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
    params.sigma = 2e-2;
    params.refrad = 0.5; % radius of reference trajectory 
    params.friction = 1; 
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "random"; % random, interval, origin
    params.numPoints = 500;
    params.interval = 1e-3;
    params.solver = "nonstifflr"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstiff"
end