%% clear workspace
close all; clear; clc;

%% test variables
file = "exper_model/dnn_10_256_100"; %"dnnv2"; %"pinn9_5_256_5000"; "dnnv2_model" "model/dnnv2_5_128_101"
net = load(file).net; %dnnv2_10s_6_256_300; % dnnv2_256_6_800
sysParams = params_system();
ctrlParams = params_control();
trainParams = params_training();
trainParams.type = "dnn"; % "dnn3","lstm3","pinn3","dnn6","lstm6","pinn6","dnn9", "lstm9","pinn9"
ctrlParams.method = "origin"; % random, interval, origin
ctrlParams.solver = "nonstiff";
numTime = 100;
tSpan = [0,20]; % [0,5] 0:0.01:5
predInterval = tSpan(2); 

%% simulation and prediction
theta = 2*pi*rand;
rad = sqrt(rand);
ctrlParams.refx = 1; %ctrlParams.xrange*rad*cos(theta);
ctrlParams.refy = 1; %ctrlParams.yrange*rad*sin(theta);
ctrlParams.phi = 0; %2*pi*rand;
ctrlParams.a = 0.25; %+rand*0.25; % target object horizontal dimension
ctrlParams.b = 0.25+0.25; % vertical dimension rand*
% x0 = [-1; -1; 0; 0; 0] + [2; 2; 2*pi; 2*pi; 2*pi].*rand(5,1); % th0, th1, th2
% x0 = [x0(1); 0; x0(2); 0; x0(3); 0; x0(4); 0; x0(5); 0]; % th0, th0d, th1, th1d, th2, th2d
x0 = zeros(10,1);
tic
y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
tEnd = toc;
t = y(:,1);
x = y(:,2:16);
obj = y(:,22:25);
ref = y(:,22:23);
[xp, rmseErr, refTime, tPred] = evaluate_single(net, t, x, obj, ctrlParams, trainParams, tSpan, predInterval, numTime, trainParams.type,1);

%% Plots
plot_compared_states(t,x,t,xp,"position",y(:,27:31));
plot_compared_states(t,x,t,xp,"velocity",y(:,[19 17 18]));
if trainParams.type == "pgnn" || trainParams.type == "dnnv2"
    plot_compared_states(t,x,t,xp,"acceleration",y(:,[19 17 18]));
end
% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,~,~,xend,yend] = ForwardKinematics(x(:,1:5),sysParams,"normal");
[~,~,~,~,~,~,~,~,xpend,ypend] = ForwardKinematics(xp(:,1:5),sysParams,"normal");
plot_endeffector(t,[xend yend],[xpend ypend],y(:,22:23)) %y(:,15:16)
% make image and video
% tPred = [1,25];
% MakeImage(ctrlParams, sysParams, t, x, xp, ref, tPred)
MakeImage(ctrlParams, sysParams, t, x, xp, ref, [tPred(1) 10])
% MakeVideo(ctrlParams, sysParams, t, x, xp, ref,[xend yend],[xpend ypend], tPred)
disp(mean(rmseErr,'all'))
% disp(Predtime)
disp(tEnd)

%% evaluate for four states
tSpan = [0,20];
predInterval = tSpan(2);
numCase = 20;
numTime = 500;
[avgErr,errs,tPred,tSim] = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, trainParams.type,1,1);
% avgErr = evaluate_model_with_4_states(net, sysParams, ctrlParams, trainParams, f1Max, tSpan, predInterval, numCase, numTime, trainParams.type);
disp(avgErr)

%% numerical gradients
% q1ddn = gradient(y(:,7),y(:,1));
q1ddnn = rdiff_kalman(xp(:,6),t,[], 'ncp');
q1dd = rdiff_kalman(x(:,6),t,[], 'ncp');
q2ddnn = rdiff_kalman(xp(:,7),t,[], 'ncp');
q2dd = rdiff_kalman(x(:,7),t,[], 'ncp');
q3ddnn = rdiff_kalman(xp(:,8),t,[], 'ncp');
q3dd = rdiff_kalman(x(:,8),t,[], 'ncp');
q4ddnn = rdiff_kalman(xp(:,9),t,[], 'ncp');
q4dd = rdiff_kalman(x(:,9),t,[], 'ncp');
% q5ddn = gradient(y(:,11),y(:,1));
q5ddnn = rdiff_kalman(xp(:,10),t,[], 'ncp');
q5dd = rdiff_kalman(x(:,10),t,[], 'ncp');

figure;
plot(y(:,1),y(:,12),y(:,1),q1ddnn,t,q1dd)
legend("real","predictions","numerical")
figure;
plot(y(:,1),y(:,13),y(:,1),q2ddnn,t,q2dd)
legend("real","predictions","numerical")
figure;
plot(y(:,1),y(:,14),y(:,1),q3ddnn,t,q3dd)
legend("real","predictions","numerical")
figure;
plot(y(:,1),y(:,15),y(:,1),q4ddnn,t,q4dd)
legend("real","predictions","numerical")
figure;
plot(y(:,1),y(:,16),y(:,1),q5ddnn,t,q5dd) % ,y(:,1),grad 0.5*q1ddn+0.5*q1ddn2
% plot(y(:,1),smoothdata(y(:,12)),y(:,1),smoothdata(grad),y(:,1),smoothdata(grad2))
legend("real","predictions","numerical")

F_true = physics_law([x(:,1:10)';q1dd';q2dd';q3dd';q4dd';q5dd'],sysParams);
F_pred = physics_law([xp(:,1:10)';q1ddnn';q2ddnn';q3ddnn';q4ddnn';q5ddnn'],sysParams);
Ferr = mean((F_true-F_pred).^2);

%%
% dnnErr = errs;
% pgnnErr = errs;
pinnErr = errs;

%% plot training curves
dnnLoss = load(file).info_dnn_128_6_4000.TrainingLoss;
pgnnLoss = load(file).info_pgnn_128_3_7000.TrainingLoss;
pinnLoss = zeros(size(pgnnLoss)); % load(file).loss;

figure('Position',[500,100,500,400]); 
iter = 1:100000;
% smoothdata(dnnLoss(iter),'gaussian')
plot(iter,dnnLoss(iter), "LineWidth",2,"DisplayName","DNN");
hold on;
plot(iter,pinnLoss(iter), "LineWidth",2,"DisplayName","PINN","LineStyle","--");
hold on;
plot(iter,pgnnLoss(iter), "LineWidth",2,"DisplayName","PGNN","LineStyle","-.");
xlabel("Iteration","FontName","Arial")
ylabel("Loss","FontName","Arial")
legend("location","northeast","FontName","Arial")
title("Training Loss","FontName","Arial");
set(gca, 'FontSize', 15);

%% plot average error

figure('Position',[500,100,800,400],'Color','White');
t = linspace(0,20,100);
plot(t,mean(dnnErr,1), "LineWidth",2,"DisplayName","DNN");
hold on
plot(t,mean(pinnErr,1), "LineWidth",2, "DisplayName","PINN","LineStyle","--");
hold on
plot(t,mean(pgnnErr,1), "LineWidth",2,"DisplayName","PGNN","LineStyle","-.");
title("Average Prediction Errors");
xlabel("Time (s)");
ylabel("RMSE");
legend("Location","northeast");
set(gca,"FontName","Arial", "FontSize", 15);

%% functions
function plot_endeffector(t,x,xp,refs)
    Idx = find(t >= 1, 1, 'first');
    refClr = "blue";
    figure('Position',[500,100,800,800]);
    tiledlayout("vertical","TileSpacing","tight")
    plot(x(:,1),x(:,2),'Color',refClr,'LineWidth',2);
    hold on 
    plot(xp(Idx:end,1),xp(Idx:end,2),'r--','LineWidth',2)
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'m:','LineWidth',2);
    end
    hold on
    plot(xp(Idx,1),xp(Idx,2),'Marker',"o",'MarkerFaceColor','g','MarkerSize',10)
    axis padded
    grid on
    legend("Ground Truth","Prediction","Reference Trajectory","Predictions Start (1 sec)","Location","best","FontName","Arial",'FontSize',15);
    % title('End Effector Path')
    % xline(1,'k--','LineWidth',2);
    ylabel("Y [m]");
    xlabel("X [m]");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');
end