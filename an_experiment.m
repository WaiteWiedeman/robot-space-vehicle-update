%% clear workspace
close all;
clear; 
clc;

%% parameters
sysParams = params_system();
ctrlParams = params_control();
trainParams = params_training();
trainParams.numSamples = 100;
trainParams.type = "dnn"; % "dnn6","pinn6","dnn9","pinn9"
trainParams.numLayers = 8;
trainParams.numNeurons = 256;
trainParams.datasource = "odes";
% trainParams.numEpochs = 1;
% trainParams.numEpochs = 400;
% trainParams.lrDropEpoch = 100;
modelFile = "model\"+trainParams.type+"_"+num2str(trainParams.numLayers)+"_"+num2str(trainParams.numNeurons)+"_"+num2str(trainParams.numSamples)+".mat";

%% generate samples
if ~exist("\data\", 'dir')
   mkdir("data");
end
dataFile = generate_samples(sysParams, ctrlParams, trainParams);
% plot(sort(fMaxRange));
% histogram(sort(fMaxRange),trainParams.numSamples)

%% plot data
ds = load('trainingSamples.mat');
ind = randi(length(ds.samples));
data = load(ds.samples{ind,1}).state;
y = data';

% plot states, forces, and states against reference
plot_states(y(:,1),y(:,2:16),"position",y(:,27:31));
plot_states(y(:,1),y(:,2:16),"velocity",y(:,27:31));
plot_states(y(:,1),y(:,2:16),"acceleration",y(:,27:31));
plot_forces(y(:,1),y(:,17:21));

% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,~,~,xend,yend] = ForwardKinematics(y(:,2:6),sysParams,"normal");
plot_endeffector([xend yend],y(:,22:23)) %y(:,15:16)

%% train model
if ~exist("\model\", 'dir')
   mkdir("model");
end

switch trainParams.type
    case "dnn6"
        [xTrain,yTrain,layers,options] = train_dnn_model_4(dataFile, trainParams);
        [net,info] = trainNetwork(xTrain,yTrain,layers,options);
        plot(layers)
    case "pinn6"
        monitor = trainingProgressMonitor;
        output = train_pinn_model_4(dataFile, trainParams,sysParams,ctrlParams,monitor);
        net = output.trainedNet;
    case "dnn"
        [net, info] = train_dnn_model(dataFile, trainParams);
    case "dnnv2"
        [xTrain,yTrain,layers,options] = train_dnnv2_model(dataFile, trainParams);
        figure;
        plot(layers)
        [net,info] = trainNetwork(xTrain,yTrain,layers,options);
    case "dnnv2_10s"
        [xTrain,yTrain,layers,options] = train_dnnv2_model_10state(dataFile, trainParams);
        figure;
        plot(layers)
        [net,info] = trainNetwork(xTrain,yTrain,layers,options);
    case "pinn9"
        monitor = trainingProgressMonitor;
        output = train_pinn_model_9(dataFile, trainParams,sysParams,ctrlParams,monitor);
        net = output.trainedNet;
    case "pinn"
        monitor = trainingProgressMonitor;
        output = train_pinn_model(dataFile, trainParams,sysParams,ctrlParams,monitor);
        info = monitor.MetricData.Loss;
        net = output.trainedNet;
    case "pgnn"
        % [xTrain,yTrain,layers,options] = train_pgnn_model(dataFile, trainParams);
        % figure;
        % plot(layers)
        % [net,info] = trainNetwork(xTrain,yTrain,layers,options);
        [net, info] = train_pgnn_model(dataFile, trainParams, sysParams);
    otherwise
        disp("unspecified type of model.")
end

% training with numeric array data
% trainLoss = info.TrainingLoss;
save(modelFile, 'net','info');
% disp(info)
% save("trainingoutput",'monitor')

%% test variables
%{
file = "best_dnn_models_5";
net = load(file).dnn9_256_4_800; % trainedNetwork dnn9_4_512_1500
sysParams = params_system();
ctrlParams = params_control();
trainParams = params_training();
trainParams.type = "dnn9"; % "dnn3","lstm3","pinn3","dnn6","lstm6","pinn6","dnn9", "lstm9","pinn9"
ctrlParams.method = "origin"; % random, interval, origin
ctrlParams.solver = "stiffhr";
numTime = 100;
tSpan = [0,25]; % [0,5] 0:0.01:5
predInterval = tSpan(2); 

%% simulation and prediction
x0 = [0; 0; 0; 0; 0; 0]; % th0, th0d, th1, th1d, th2, th2d
theta = 2*pi*rand;
rad = sqrt(rand);
ctrlParams.refx = ctrlParams.a*rad*cos(theta);
ctrlParams.refy = ctrlParams.b*rad*sin(theta);
% x0 = [-1; 0; 0] + [2; 2*pi; 2*pi].*rand(3,1); % th0, th1, th2
% x0 = [x0(1); 0; x0(2); 0; x0(3); 0]; % th0, th0d, th1, th1d, th2, th2d
y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
t = y(:,1);
x = y(:,2:10);
ref = y(:,15:19);
[xp, rmseErr, refTime] = evaluate_single(net, t, x, ctrlParams, trainParams, tSpan, predInterval, numTime, trainParams.type,1);

%% Plots
plot_compared_states(t,x,t,xp,"position",y(:,[19 17 18]));
plot_compared_states(t,x,t,xp,"velocity",y(:,[19 17 18]));
plot_compared_states(t,x,t,xp,"acceleration",y(:,[19 17 18]));
% solve forward kinematics and plot end effector position
[~,~,~,~,~,~,xend,yend] = ForwardKinematics(x(:,1:3),sysParams);
[~,~,~,~,~,~,xpend,ypend] = ForwardKinematics(xp(:,1:3),sysParams);
plot_endeffector([xend yend],[xpend ypend],y(:,15:16)) %y(:,15:16)
% make image and video
tPred = [1,45];
MakeImage(ctrlParams, sysParams, t, x, xp, ref, tPred)
% MakeVideo(ctrlParams, sysParams, t, x, xp, ref,[xend yend],[xpend ypend], tPred)
disp(mean(rmseErr,'all'))

%% evaluate for four states
tSpan = [0,20];
predIntervel = 20;
numCase = 100;
numTime = 100;
avgErr = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, trainParams.type,1,1);
% avgErr = evaluate_model_with_4_states(net, sysParams, ctrlParams, trainParams, f1Max, tSpan, predInterval, numCase, numTime, trainParams.type);
disp(avgErr)
%}
%% functions
function plot_forces(t,u)
    figure('Position',[500,100,800,800]);
    plot(t,u(:,1),'k-',t,u(:,2),'b-',t,u(:,3),'g-',t,u(:,4),'m-',t,u(:,5),'c-','LineWidth',2);
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
            ylabel(labels(i),"Interpreter","latex");
            set(get(gca,'ylabel'),'rotation',0);
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
    plot(x(:,1),x(:,2),'b-o','LineWidth',2);
    if refs ~= 0
        hold on
        plot(refs(:,1),refs(:,2),'*','LineWidth',2);
    end
    axis padded
    % xline(1,'k--','LineWidth',2);
    ylabel("Y");
    xlabel("X");
    set(get(gca,'ylabel'),'rotation',0);
    set(gca, 'FontSize', 15);
    set(gca, 'FontName', 'Arial');

end
