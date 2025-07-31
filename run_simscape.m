function y = run_simscape(mdl,ctrlParams) %simIn,ctrlParams
out = sim(mdl);
t = out.tout;
% disp(size(t))
numTime = length(t);
y = zeros(numTime, 21);
y(:,1) = t; % t
y(:,2) = out.yout{1}.Values.Data; % xv
y(:,3) = out.yout{2}.Values.Data; % yv
y(:,4) = out.yout{3}.Values.Data; % alv
y(:,5) = out.yout{4}.Values.Data; % th1
y(:,6) = out.yout{5}.Values.Data; % th2
y(:,7) = out.yout{6}.Values.Data; % xvdot
y(:,8) = out.yout{7}.Values.Data; % yvdot
y(:,9) = out.yout{8}.Values.Data; % alvdot
y(:,10) = out.yout{9}.Values.Data; % th1dot
y(:,11) = out.yout{10}.Values.Data; % th2dot
y(:,12) = out.yout{11}.Values.Data; % xvddot
y(:,13) = out.yout{12}.Values.Data; % yvddot
y(:,14) = out.yout{13}.Values.Data; % alvddot
y(:,15) = out.yout{14}.Values.Data; % th1ddot
y(:,16) = out.yout{15}.Values.Data; % th2ddot
y(:,17) = out.yout{16}.Values.Data; % ux
y(:,18) = out.yout{17}.Values.Data; % uy
y(:,19) = out.yout{18}.Values.Data; % t0
y(:,20) = out.yout{19}.Values.Data; % t1
y(:,21) = out.yout{20}.Values.Data; % t2
y(:,22) = out.yout{21}.Values.Data; % X end desired
y(:,23) = out.yout{22}.Values.Data; % Y end desired
y(:,24) = ctrlParams.refx*ones(numTime, 1);
y(:,25) = ctrlParams.refy*ones(numTime, 1);
y(:,26) = ctrlParams.phi*ones(numTime, 1);
y(:,27) = out.yout{23}.Values.Data; % desired vehicle x position
y(:,28) = out.yout{24}.Values.Data; % desired vehicle y position
y(:,29) = out.yout{25}.Values.Data; % desired vehicle pitch angle
y(:,30) = out.yout{26}.Values.Data; % Th1 desired
y(:,31) = out.yout{27}.Values.Data; % Th2 desired
y = select_samples(ctrlParams, t, y);
end

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