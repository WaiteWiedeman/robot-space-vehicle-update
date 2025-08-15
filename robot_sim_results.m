%% clear workspace
close all; clear; clc;

%% Two-link Planar Robot on Cart Dynamics System Parameters
sysParams = params_system();
ctrlParams = params_control();
% ctrlParams.method = "origin";
% ctrlParams.solver = "nonstiff"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstifflr" or "nonstiff"
% ctrlParams.noise = 0;
% ctrlParams.sigma = 1e-2;
tSpan = [0,20]; %[0,20]; %0:0.01:15;
% ctrlParams.PID3(3) = ctrlParams.PID3(3)+100;
% ctrlParams.dt_control = 0.001; % time step difference in which new control action is calculated
% ctrlParams.Pf = 0.1;
% ctrlParams.Flim = 100;
% ctrlParams.Tlim = 50;

%% generate random target and initial state
theta = 2*pi*rand;
rad = sqrt(rand);
ctrlParams.refx = ctrlParams.xrange*rad*cos(theta);
ctrlParams.refy = ctrlParams.yrange*rad*sin(theta);
ctrlParams.phi = 2*pi*rand;
ctrlParams.a = 0.25+rand*0.25; % target object horizontal dimension
ctrlParams.b = 0.25+rand*0.25; % vertical dimension
x0 = [-1; -1; 0; 0; 0] + [2; 2; 2*pi; 2*pi; 2*pi].*rand(5,1); % th0, th1, th2
x0 = [x0(1); 0; x0(2); 0; x0(3); 0; x0(4); 0; x0(5); 0]; % th0, th0d, th1, th1d, th2, th2d
% x0 = zeros(10,1); % th0, th0d, th1, th1d, th2, th2d

%% run simulation 
tic
y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
tEnd = toc;
disp(tEnd)

%% plot states, forces, and states against reference
% plot states, forces, and states against reference
plot_states(y(:,1),y(:,2:16),"position",y(:,27:31));
plot_states(y(:,1),y(:,2:16),"velocity",y(:,27:31));
plot_states(y(:,1),y(:,2:16),"acceleration",y(:,27:31));
plot_forces(y(:,1),y(:,17:21));

% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:6),sysParams,"normal");
plot_endeffector([xend yend],y(:,22:23)) %y(:,15:16)

info1 = lsiminfo(y(:,2),y(:,1));
info2 = lsiminfo(y(:,3),y(:,1));
info3 = lsiminfo(y(:,4),y(:,1));

% MakeVideo(ctrlParams,sysParams, y(:,1), y(:,2:6), y(:,22:23), [0,1]);
MakeImage(ctrlParams,sysParams, y(:,1), y(:,2:6), 0, y(:,22:23), tSpan)

% disp(xend(end))
% disp(yend(end))
% disp(y(end,2)-y(end,19))
% disp(y(end,3)-y(end,17))
% disp(y(end,4)-y(end,18))

%% noise plot
figure('Position',[500,100,800,800]);
plot(y(:,1),y(:,32:41),'LineWidth',2);
xlabel('Time (s)')
ylabel('Noise');
legend("$x_v$","$y_v$","$\alpha_v$","$\theta_1$","$\theta_2$","$\dot{x}_v$","$\dot{y}_v$"...
    ,"$\dot{\alpha}_v$","$\dot{\theta}_1$","$\dot{\theta}_2$","Interpreter","latex");
set(gca, 'FontSize', 15);
set(gca, 'FontName', 'Arial');

%% torques plot
t=y(:,1);
figure('Position',[500,100,800,800]);
plot(t,y(:,20),'c-',t,y(:,21),'b-',t,y(:,42),'g-',t,y(:,43),'m-','LineWidth',2);
xlabel("Time (s)");
ylabel("Torque (N*m)");
% set(get(gca,'ylabel'),'rotation',0);
grid on
set(gca, 'FontSize', 15);
set(gca, 'FontName', "Arial")
legend("$\tau_1$","$\tau_2$","$\tau_{f_1}$","$\tau_{f_2}$","Interpreter","latex");

%% viscous friction
th1d = linspace(-5,5,100);
th2d = linspace(-5,5,100);
T_f = zeros(2,length(th1d));
for i = 1:length(th1d)
    T_f(:,i) = friction(th1d(i),th2d(i),sysParams);
end
figure('Position',[500,100,800,800]);
plot(th1d,T_f(1,:),'g-',th2d,T_f(2,:),'m-','LineWidth',2);
xlabel("Angular velocity (rad/s)");
ylabel("Friction Torque (N*m)");
% set(get(gca,'ylabel'),'rotation',0);
grid on
set(gca, 'FontSize', 15);
set(gca, 'FontName', "Arial")
legend("$\tau_{f_1}$","$\tau_{f_2}$","Interpreter","latex");

%% run simscape model and plot states, forces, and states against reference for simscape model
mdl = "robot_model.slx";
% theta = 2*pi*rand;
% rad = sqrt(rand);
% ctrlParams.refx = ctrlParams.xrange*rad*cos(theta);
% ctrlParams.refy = ctrlParams.yrange*rad*sin(theta);
% ctrlParams.phi = 2*pi*rand;
% ctrlParams.a = 0.25+rand*0.25; % target object horizontal dimension
% ctrlParams.b = 0.25+rand*0.25; % vertical dimension
% ctrlParams.method = "interval";
y_simscape = run_simscape(mdl,ctrlParams); %simIn,ctrlParams

% plot states, forces, and states against reference
plot_states(y_simscape(:,1),y_simscape(:,2:16),"position",y_simscape(:,27:31));
plot_states(y_simscape(:,1),y_simscape(:,2:16),"velocity",y_simscape(:,27:31));
plot_states(y_simscape(:,1),y_simscape(:,2:16),"acceleration",y_simscape(:,27:31));
plot_forces(y_simscape(:,1),y_simscape(:,17:21));

% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,~,~,xend,yend] = ForwardKinematics(y_simscape(:,2:6),sysParams);
plot_endeffector([xend yend],y_simscape(:,22:23)) %y(:,15:16)

%% Set parameters to Tune PID w/ GA 
N_monte_carlo = 10;
tSpan = [0,25];
Ts_lim = 15;
ctrlParams.solver = "GA";
myObj = @(gene) fitnessfun(gene, N_monte_carlo,tSpan,Ts_lim,ctrlParams,sysParams);
% GA parameters
Pc = 0.8;
fitfun = @(x) myObj(x);
PopSize = 100;
MaxGens = 200;
nvars   = 9;
A       = [];
b       = [];
Aeq     = [];               
beq     = [];
% lb      = [1 0 0.1 1 0 0.1 1 0 0.1 1 0 0.1 1 0 0.1 10 0 1 10 0 1 10 0 1 10 0 1 10 0 1 0];
% ub      = [20 1 10 20 1 10 20 1 10 20 1 10 20 1 10 100 5 30 100 5 30 100 5 30 100 5 30 100 5 30 0.5];
lb = [ ...          
    repmat([700 200 2000], 1, 2), ...   % 5 PIDs
    [700 100 300], ...
    %repmat([500 100 300], 1, 2), ...
    ];                      

ub = [ ...
    repmat([1000 500 3000], 1, 2), ...
    [1500 500 1000], ...
    %repmat([1000 500 1000], 1, 2), ...
    ];
nonlcon = [];
options = optimoptions('ga', 'PopulationSize', PopSize, 'MaxGenerations',...
    MaxGens,'PlotFcn',{@gaplotbestf,@gaplotscores},'UseParallel',true);
options.CrossoverFraction = Pc;
options.EliteCount = 3;

%% GA to search for near-optimal solution
start = datetime
[BestChrom, fval, exitflag, output] = ga(fitfun, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);
stop = datetime
disp(stop-start)
save("best_controller.mat","BestChrom")

%% simulate PID controller tuned by GA
ctrlParams.PID1 = BestChrom(1:3);
ctrlParams.PID2 = BestChrom(4:6);
ctrlParams.PID3 = BestChrom(7:9);
% ctrlParams.PID4 = BestChrom(10:12);
% ctrlParams.PID5 = BestChrom(13:15);

%% coulomb friction
v = linspace(-5,5,10000);
fc = coulomb_friction(v, sysParams, ctrlParams.friction);
idx = find(v >= 0, 1, 'first');
slopeangle = atan((fc(idx))/(v(idx)))*180/pi;
figure('Position',[500,100,800,800]);
plot(v,fc,'b-','LineWidth',2);
xlabel("velocity (m/s)");
ylabel('Coulomb Friction Force (N)');
set(gca, 'FontSize', 15);
set(gca, 'FontName', 'Arial');
disp(slopeangle)

%% random points
t=0:0.01:5;
x = ctrlParams.a*cos((2*pi/5)*t);
y = ctrlParams.b*sin((2*pi/5)*t);

numCase = 100;
theta = linspace(0,2*pi,numCase);
rad = linspace(0,1,numCase);

for i = 1:numCase
    % theta = 2*pi*rand;
    % rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.a*rad(i)*cos(theta(i));
    ctrlParams.refy = ctrlParams.b*rad(i)*sin(theta(i));
    Xd(i) = ctrlParams.refx;
    Yd(i) = ctrlParams.refy;
end

figure('Position',[500,100,800,800]);
plot(x,y,'k-',Xd,Yd,'b*','LineWidth',1)
axis padded
ylabel("Y");
xlabel("X");
set(get(gca,'ylabel'),'rotation',0);
set(gca, 'FontSize', 15);
set(gca, 'FontName', 'Arial');
legend("Objective Bounds", "Random Objective Point")

%% rotating ellipse
time = linspace(0,5,10);
th = linspace(0,2*pi,100);
beta = (2*pi/5)*time; %[pi/4, pi/2, 3*pi/4]; %linspace(0,2*pi,10);
h = 1; k = 1; a = 4; b = 3;
figure;
for i = 1:length(beta)
    x = h + a*cos(th)*cos(beta(i)) - b*sin(th)*sin(beta(i)); 
    y = k + b*sin(th)*cos(beta(i)) + a*cos(th)*sin(beta(i));
    plot(x,y,'o-')
    axis square
    hold on
end

%%
% q1ddn = gradient(y(:,7),y(:,1));
% q1ddn = rdiff_kalman(y(:,7),y(:,1),[], 'ncp');
% q2ddn = rdiff_kalman(y(:,8),y(:,1),[], 'ncp');
% q3ddn = rdiff_kalman(y(:,9),y(:,1),[], 'ncp');
% q4ddn = rdiff_kalman(y(:,10),y(:,1),[], 'ncp');
% % q5ddn = gradient(y(:,11),y(:,1));
% q5ddn = rdiff_kalman(y(:,11),y(:,1),[], 'ncp');
q1d = interp1(y(:,1), y(:,7), y(:,1), 'spline');
q1dd = gradient(q1d, y(:,1));
q2d = interp1(y(:,1), y(:,8), y(:,1), 'spline');
q2dd = gradient(q2d, y(:,1));
q3d = interp1(y(:,1), y(:,9), y(:,1), 'spline');
q3dd = gradient(q3d, y(:,1));
q4d = interp1(y(:,1), y(:,10), y(:,1), 'spline');
q4dd = gradient(q4d, y(:,1));
q5d = interp1(y(:,1), y(:,11), y(:,1), 'spline');
q5dd = gradient(q5d, y(:,1));
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,12),y(:,1),q1dd)
legend("real","numerical")
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,13),y(:,1),q2dd)
legend("real","numerical")
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,14),y(:,1),q3dd)
legend("real","numerical")
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,15),y(:,1),q4dd)
legend("real","numerical")
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,16),y(:,1),q5dd) % ,y(:,1),grad 0.5*q1ddn+0.5*q1ddn2
% plot(y(:,1),smoothdata(y(:,12)),y(:,1),smoothdata(grad),y(:,1),smoothdata(grad2))
legend("real","numerical")
disp("q1dd error: "+num2str(mean(y(:,12)-q1dd,'all')))
disp("q2dd error: "+num2str(mean(y(:,13)-q2dd,'all')))
disp("q3dd error: "+num2str(mean(y(:,14)-q3dd,'all')))
disp("q4dd error: "+num2str(mean(y(:,15)-q4dd,'all')))
disp("q5dd error: "+num2str(mean(y(:,16)-q5dd,'all')))

%%
figure('Position',[500,200,800,800]);
plot(y(:,1),y(:,2),'b-o','LineWidth',2);
ylabel("$\theta_0$","Interpreter","latex");
set(get(gca,'ylabel'),'rotation',0);
set(gca, 'FontSize', 15);
set(gca, 'FontName', "Arial")
xlabel("Time (s)");

%% functions
function plot_forces(t,u)
    figure('Position',[500,100,800,800]);
    plot(t,u(:,1),'k-',t,u(:,2),'b-',t,u(:,3),'g-',t,u(:,4),'m-',t,u(:,5),'c-','LineWidth',2);
    xlabel("Time (s)");
    ylabel("Force (N)");
    % set(get(gca,'ylabel'),'rotation',0);
    grid on
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', "Arial")
    legend("$u_x$","$u_y$","$\tau_0$","$\tau_1$","$\tau_2$","Interpreter","latex");
end

function plot_states(t,x,flag,refs)
labels= ["$x_v$","$y_v$","$\alpha_v$","$\theta_1$","$\theta_2$","$\dot{x}_v$","$\dot{y}_v$","$\dot{\alpha}_v$","$\dot{\theta}_1$","$\dot{\theta}_2$",...
    "$\ddot{x}_v$","$\ddot{y}_v$","$\ddot{\alpha}_v$","$\ddot{\theta}_1$","$\ddot{\theta}_2$"];
switch flag
    case "position"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 1:numState-10
            nexttile
            plot(t,x(:,i),'b-',t,refs(:,i),'k:','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            if i == 1 || i ==2
                ylabel(labels(i) + "[m]","Interpreter","latex");
            else
                ylabel(labels(i) + "[rad]","Interpreter","latex");
            end
            % ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            grid on
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-10
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Reference","Location","eastoutside","FontName","Arial");
    case "velocity"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 6:numState-5
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            grid on
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-5
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Location","eastoutside","FontName","Arial");
    case "acceleration"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 11:numState
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            % xline(1,'k--', 'LineWidth',1);
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
            grid on
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState
                xlabel("Time (s)");
            end
        end
        % legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
end
end

function plot_endeffector(x,refs)
    refClr = "blue";
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    plot(x(:,1),x(:,2),'Color',refClr,'LineWidth',2);
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'m:','LineWidth',2);
    end
    axis padded
    grid on
    legend("Ground Truth","Reference Trajectory","Location","best","FontName","Arial",'FontSize',15);
    % title('End Effector Path')
    % xline(1,'k--','LineWidth',2);
    ylabel("Y [m]");
    xlabel("X [m]");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');
end

function [fit]  = fitnessfun(gene, N_monte_carlo,tSpan,Ts_lim,ctrlParams,sysParams) %,Rise_thresh ,POlim,Ts_lim

ctrlParams.PID1 = gene(1:3);
ctrlParams.PID2 = gene(4:6);
ctrlParams.PID3 = gene(7:9);
% ctrlParams.PID4 = gene(10:12);
% ctrlParams.PID5 = gene(13:15);

e1 = zeros(1,N_monte_carlo);
e2 = zeros(1,N_monte_carlo);
e3 = zeros(1,N_monte_carlo);
e4 = zeros(1,N_monte_carlo);
e5 = zeros(1,N_monte_carlo);
exend = zeros(1,N_monte_carlo);
eyend = zeros(1,N_monte_carlo);

for sim_n = 1:N_monte_carlo
    theta = 2*pi*rand;
    rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.xrange*rad*cos(theta);
    ctrlParams.refy = ctrlParams.yrange*rad*sin(theta);
    ctrlParams.phi = 2*pi*rand;
    ctrlParams.a = 0.25+rand*0.25; % target object horizontal dimension
    ctrlParams.b = 0.25+rand*0.25; % vertical dimension
    x0 = [-1; -1; 0; 0; 0] + [2; 2; 2*pi; 2*pi; 2*pi].*rand(5,1); % th0, th1, th2
    x0 = [x0(1); 0; x0(2); 0; x0(3); 0; x0(4); 0; x0(5); 0]; % th0, th0d, th1, th1d, th2, th2d
    % x0 = zeros(10,1); % th0, th0d, th1, th1d, th2, th2d
    try
        y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
    catch
        fit = 1e6;
        return;
    end
    [~,~,~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:6),sysParams,"normal");
    
    if y(end,1) < tSpan(2) 
        disp("loop bailed: too much time")
        fit = 1e6;
        return;
    end

    Idx = find(y(:,1) > Ts_lim, 1, 'first'); % start where force stop acting
    
    e1(sim_n) = mean((y(Idx:end,2) - y(Idx:end,27)).^2,'all');
    e2(sim_n) = mean((y(Idx:end,3) - y(Idx:end,28)).^2,'all');
    e3(sim_n) = mean((y(Idx:end,4) - y(Idx:end,29)).^2,'all');
    e4(sim_n) = mean((y(Idx:end,5) - y(Idx:end,30)).^2,'all');
    e5(sim_n) = mean((y(Idx:end,6) - y(Idx:end,31)).^2,'all');
    exend(sim_n) = mean((y(Idx:end,22) - xend(Idx:end)).^2,'all');
    eyend(sim_n) = mean((y(Idx:end,23) - yend(Idx:end)).^2,'all');

end
disp('monte carlo done')
disp(mean(e1))
disp(mean(e2))
disp(mean(e3))
disp(mean(e4))
disp(mean(e5))
disp(mean(exend))
disp(mean(eyend))

fit = 1e3*(1*(mean(e1)+mean(e2)+mean(e3)+mean(exend)+mean(eyend))); % +mean(e4)+mean(e5)

end
