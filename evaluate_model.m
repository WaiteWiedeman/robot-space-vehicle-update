function [avgErr,errs,tPred,tSim] = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, type, show, initTime)
    % evaluate time span, larger time span will increase the simulation
    % time when complicated friction involved
    theta = linspace(0,2*pi,numCase);
    rad = linspace(0,1,numCase);
    x0el = linspace(-1,1,numCase);
    a = linspace(0,0.25,numCase);

    % reference time points 
    switch trainParams.type
        case {"dnn3", "lstm3", "pinn3", "pirn3"} 
            errs = zeros(3*numCase, numTime);
        case {"dnn6", "lstm6", "pinn6", "pirn6"} 
            errs = zeros(6*numCase, numTime);
        case {"dnn", "dnnv2_10s", "pinn", "pgnn"}
            errs = zeros(10*numCase, numTime);
        case {"lstm9", "pirn9", "dnnv2"} 
            errs = zeros(15*numCase, numTime);
        otherwise
            disp("unspecify type of model.")
    end
    for i = 1:numCase
        x0 = [x0el(i); 0; x0el(i); 0; theta(i); 0; theta(i); 0; theta(i); 0]; % th0, th0d, th1, th1d, th2, th2d
        ctrlParams.refx = ctrlParams.xrange*rad(i)*cos(theta(i));
        ctrlParams.refy = ctrlParams.yrange*rad(i)*sin(theta(i));
        ctrlParams.a = 0.25+a(i); % target object horizontal dimension
        ctrlParams.b = 0.5-a(i); % vertical dimension
        tic
        y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
        tEnd(i) = toc;
        t = y(:,1);
        x = y(:,2:16);
        obj = y(:,22:25);
        [xp, rmseErr, refTime,tPred(i)] = evaluate_single(net, t, x, obj, ctrlParams, trainParams, tSpan, predInterval, numTime, type, initTime);
        if show
            disp("evaluate " + num2str(i) + " th case, mean square err: " + num2str(mean(rmseErr, "all")));
        end
        switch trainParams.type
            case {"dnn3", "lstm3", "pinn3", "pirn3"} 
                errs(3*(i-1)+1:3*(i-1)+3,:) = rmseErr;
            case {"dnn6", "lstm6", "pinn6", "pirn6"} 
                errs(6*(i-1)+1:6*(i-1)+6,:) = rmseErr;
            case {"dnn", "dnnv2_10s", "pinn", "pgnn"}
                errs(10*(i-1)+1:10*(i-1)+10,:) = rmseErr(1:10,:);
            case {"lstm9", "pirn9", "dnnv2"} 
                errs(15*(i-1)+1:15*(i-1)+15,:) = rmseErr;    
            otherwise
                disp("unspecify type of model.")
        end
    end
    tSim = mean(tEnd,"all");
    tPred = mean(tPred,"all");
    avgErr = mean(errs,'all'); % one value of error for estimtation
    if show
        disp("plot time step rsme")
        figure('Position',[500,100,800,300]); 
        tiledlayout("vertical","TileSpacing","tight")
        plot(refTime,mean(errs,1),'k-','LineWidth',2);
        xlabel("Time (s)","FontName","Arial");
        ylabel("Average RMSE","FontName","Arial");
        xticks(linspace(1,tSpan(2),(tSpan(2))));
        title("Average RMSE: "+num2str(avgErr));
        set(gca, 'FontSize', 15);
    end
end



