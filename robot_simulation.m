function y = robot_simulation(tSpan, x0, sysParams, ctrlParams)
    % ODE solver
    if ctrlParams.fixedTimeStep ~= 0
        tSpan = tSpan(1):ctrlParams.fixedTimeStep:tSpan(2);
    end
    
    show = 0;
    sz = size(x0);
    noise = ctrlParams.sigma*randn(sz(1),10000);
    % noise = lowpass(noise,0.1);

    switch ctrlParams.solver
        case "nonstifflr"
            % startTime = datetime;
            % stopTime = 60*5; % end sim in 60 seconds
            % opts = odeset('RelTol',1e-2,'AbsTol',1e-2,'OutputFcn',@(t, y, flag) myOutputFcn(t, y, flag, startTime, stopTime));
            opts = odeset('AbsTol',1e-3); %,'OutputFcn',@odeplot ,'AbsTol',1e-5 ,'OutputFcn',@odeplot
            if show
                figure;
                opts = odeset(opts,'OutputFcn',@odeplot);
            end
            [t,x] = ode23(@(t,x) robot_system(t, x, noise, sysParams, ctrlParams), tSpan, x0,opts);
        case "nonstiffhr"
            % startTime = datetime;
            % stopTime = 60*5; % end sim in 60 seconds
            % opts = odeset('OutputFcn',@(t, y, flag) myOutputFcn(t, y, flag, startTime, stopTime));
            opts = odeset('MaxStep',ctrlParams.dt_control); % ,'OutputFcn',@odeplot 'RelTol',1e-2,
            if show
                figure('Position',[500,200,800,800]);
                opts = odeset(opts,'OutputFcn',@odeplot);
            end
            [t,x] = ode45(@(t,x) robot_system(t, x, noise, sysParams, ctrlParams), tSpan, x0,opts);
        case "stiffhr"
            opts = odeset('RelTol',1e-7,'AbsTol',1e-9);
            if show
                figure;
                opts = odeset(opts,'OutputFcn',@odeplot);
            end
            [t,x] = ode15s(@(t,x) robot_system(t, x, noise, sysParams, ctrlParams), tSpan, x0,opts); 
        case "stifflr"
            opts = odeset(); %'RelTol',1e-4,'AbsTol',1e-5
            if show
                figure;
                opts = odeset(opts,'OutputFcn',@odeplot);
            end
            [t,x] = ode15s(@(t,x) robot_system(t, x, noise, sysParams, ctrlParams), tSpan, x0,opts);
        case "GA"
            startTime = datetime;
            stopTime = 60*2; % end sim in 60 seconds
            opts = odeset('MaxStep',ctrlParams.dt_control,'OutputFcn', @(t, y, flag) myOutputFcn(t, y, flag, startTime, stopTime));
            if show
                figure;
                opts = odeset(opts,'OutputFcn',@odeplot);
            end
            [t,x] = ode45(@(t,x) robot_system(t, x, noise, sysParams, ctrlParams), tSpan, x0, opts);
    end
    % [t,x] = select_samples(ctrlParams, t, x);
    numTime = length(t);
    y = zeros(numTime,43);
    for i = 1 : numTime
        [Xd, Yd, Xdd, Ydd, Xv, Xvd, Yv, Yvd] = referenceTrajectory(t(i), ctrlParams,sysParams);
        [Alv,Th1,Th2,Alvd,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xv,Xvd,Yv,Yvd);
        F = force_function(t(i), x(i,:)', Xv, Xvd, Yv, Yvd, Alv, Th1, Th2, Alvd, Om1, Om2, noise, ctrlParams);
        if ctrlParams.friction
            T_f = friction(x(i,8),x(i,10),sysParams);
        else
            T_f = zeros(2,1);
        end
        xdot = robot_xdot(x(i,:), F, T_f, sysParams);
        y(i,1) = t(i); % t
        y(i,2) = x(i, 1); % xv
        y(i,3) = x(i, 3); % yv
        y(i,4) = x(i, 5); % alv
        y(i,5) = x(i, 7); % th1
        y(i,6) = x(i, 9); % th2
        y(i,7) = x(i, 2); % xvdot
        y(i,8) = x(i, 4); % yvdot
        y(i,9) = x(i, 6); % alvdot
        y(i,10) = x(i, 8); % th1dot
        y(i,11) = x(i, 10); % th2dot
        y(i,12) = xdot(2); % xvddot
        y(i,13) = xdot(4); % yvddot
        y(i,14) = xdot(6); % alvddot
        y(i,15) = xdot(8); % th1ddot
        y(i,16) = xdot(10); % th2ddot
        y(i,17) = F(1); % ux
        y(i,18) = F(2); % uy
        y(i,19) = F(3); % t0
        y(i,20) = F(4); % t1
        y(i,21) = F(5); % t2
        y(i,22) = Xd; % X end desired
        y(i,23) = Yd; % Y end desired
        y(i,24) = ctrlParams.refx;
        y(i,25) = ctrlParams.refy;
        y(i,26) = ctrlParams.phi;
        y(i,27) = Xv; % desired vehicle x position
        y(i,28) = Yv; % desired vehicle x position
        y(i,29) = Alv; % desired vehicle pitch angle
        y(i,30) = Th1; % Th1 desired
        y(i,31) = Th2; % Th2 desired 
        % if i <= 10000
        %     y(i,32:41) = noise(:,i).*x(i,:)'; % noise
        % elseif i > 10000
            if mod(i,10000) == 0
                y(i,32:41) = noise(:,end).*x(i,:)'; % noise
            else
                div = floor(i/10000);
                y(i,32:41) = noise(:,i-div*10000).*x(i,:)'; % noise
            end
            
        % end
        y(i,42) = T_f(1); % viscous friction applied to joint 1
        y(i,43) = T_f(2); % viscous friction applied to joint 2
    end
    y = select_samples(ctrlParams, t, y);
end

% function [ts, xs] = select_samples(ctrlParams, t, x)
%     switch ctrlParams.method
%         case "random"
%             indices = randperm(length(t), ctrlParams.numPoints);
%             sortIndices = sort(indices);
%             ts = t(sortIndices);
%             xs = x(sortIndices,:);
%         case "interval"
%             ts = [t(1)];
%             xs = [x(1,:)];
%             for i = 2:length(t)
%                 if t(i)-ts(end) >= ctrlParams.interval
%                     ts = [ts;t(i)];
%                     xs = [xs;x(i,:)];
%                 end
%             end
%         otherwise
%             ts = t;
%             xs = x;
%     end
% end

function ys = select_samples(ctrlParams, t, y)
    switch ctrlParams.method
        case "random"
            indices = randperm(length(t), ctrlParams.numPoints);
            sortIndices = sort(indices);
            ys = y(sortIndices,:);
        case "interval"
            ts = [t(1)];
            ys = [y(1,:)];
            for i = 2:length(t)
                if t(i)-ts(end) >= ctrlParams.interval
                    ys = [ys;y(i,:)];
                end
            end
        otherwise
            ys = y;
    end
end

function status = myOutputFcn(t, y, flag, startTime, stopTime)
    currentTime = datetime;
    status = double(seconds(currentTime-startTime) > stopTime);
end
