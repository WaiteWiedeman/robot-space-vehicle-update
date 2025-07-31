function F = force_function(t, x, Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, noise, ctrlParams)
    persistent t_prev e_int Fp next_control_time count

    dt_control = ctrlParams.dt_control;  % 1 kHz control update

    if t == 0 || isempty(t_prev)
        t_prev = t;
        e_int = zeros(5,1);
        Fp = zeros(5,1);
        next_control_time = 0;
    end

    % Apply noise if enabled
    if ctrlParams.noise
        if isempty(count)
            count = 0;
        end
        count = count + 1;
        if count > length(noise)
            count = 1;
        end
        x = x .* (1 + noise(:,count));
    end

    % Extract states
    xv = x(1);  xvd = x(2);
    yv = x(3);  yvd = x(4);
    alv = x(5); alvd = x(6);
    th1 = x(7); th1d = x(8);
    th2 = x(9); th2d = x(10);

    % Desired positions and velocities
    q_des = [Xv; Yv; Alv; Th1; Th2];
    dq_des_feedforward = [Xvd; Yvd; Alvd; Om1; Om2];  % feedforward vel

    % Actual positions and velocities
    q_now = [xv; yv; alv; th1; th2];
    dq_now = [xvd; yvd; alvd; th1d; th2d];

    % Force and torque limits
    Flim = ctrlParams.Flim;
    Tlim = ctrlParams.Tlim;
    lims = [Flim; Flim; Tlim; Tlim; Tlim];

    PID = [ctrlParams.PID1(:)';  % Velocity control
           ctrlParams.PID2(:)';
           ctrlParams.PID3(:)';
           ctrlParams.PID4(:)';
           ctrlParams.PID5(:)'];

    if t >= next_control_time
        dt = t - t_prev;
        t_prev = t;

        %% === PID loop: Position PID -> torque/force ===
        e = q_des - q_now;
        de = dq_des_feedforward - dq_now; % (e_inner - e_prev_inner) / dt;

        u_raw = PID(:,1).*e + PID(:,2).*e_int + PID(:,3).*de;

        % Limit forces
        u_sat = max(min(u_raw, lims), -lims);
        
        if u_raw == u_sat
            e_int = e_int + e * dt;
        end

        % Filter output (1st-order lag)
        Fp = Fp + ctrlParams.Pf * (u_sat - Fp);

        next_control_time = t + dt_control;
    end

    % Output constant between updates
    F = Fp;
end
