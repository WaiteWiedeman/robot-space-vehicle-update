function [netTrained, info] = train_pgnn_model(sampleFile, trainParams, sysParams)
% Train a DNN model for learning dynamics system behvior
    % load samples and prepare training dataset
    ds = load(sampleFile);
    numSamples = trainParams.numSamples;
    
    % generate training dataset
    % Feature: 9-D initial state (x0) + the predict future time (t)
    % Label: a predicted state x = [th0,th1,th2,th0d,th1d,th2d,th0dd,th1dd,th2dd]'
    % Start from 1 sec to 4 sec with 0.5 sec step 
    initTimes = 1:trainParams.initTimeStep:6; 
    xTrain = [];
    yTrain = [];
    for i = 1:numSamples
        data = load(ds.samples{i,1}).state;
        t = data(1,:);
        x = data(2:16,:);
        obj = data(22:25,:);
        for tInit = initTimes
            initIdx = find(t > tInit, 1, 'first');
            x0 = x(:,initIdx);  % Initial state 
            obj0 = obj(:,initIdx);  % Initial state
            t0 = t(initIdx);    % Start time
            for j = initIdx+1:length(t)
                xTrain = [xTrain, [x0; obj0; t(j)-t0]];
                yTrain = [yTrain, x(:,j)];
            end
        end
    end
    disp(num2str(length(xTrain)) + " samples are generated for training.");
    xTrain = xTrain';
    yTrain = yTrain';

    % Create neural network
    numStates = 15;
    layers = [
        featureInputLayer(numStates+5, "Name", "input")
        ];
    
    numMiddle = floor(trainParams.numLayers/2);
    for i = 1:numMiddle
        layers = [
            layers
            fullyConnectedLayer(trainParams.numNeurons)
            eluLayer
        ];
    end
    if trainParams.dropoutFactor > 0
        layers = [
            layers
            dropoutLayer(trainParams.dropoutFactor)
        ];
    end
    for i = numMiddle+1:trainParams.numLayers
        layers = [
            layers
            fullyConnectedLayer(trainParams.numNeurons)
            eluLayer
        ];
    end
    
    layers = [
        layers
        fullyConnectedLayer(numStates, "Name", "output")];
        % weightedLossLayer("mse")]; % weightedLossLayer("mse")

    % layers = layerGraph(layers);
    % plot(lgraph);
    
    options = trainingOptions("adam", ...
        InitialLearnRate = trainParams.initLearningRate, ...
        MaxEpochs = trainParams.numEpochs, ...
        MiniBatchSize = trainParams.miniBatchSize, ...
        Shuffle = "every-epoch", ...
        Plots = "training-progress", ...
        Verbose = false, ...
        LearnRateSchedule = "piecewise", ...
        LearnRateDropFactor = trainParams.lrDropFactor, ...
        LearnRateDropPeriod = trainParams.lrDropEpoch);

    [netTrained, info] = trainnet(xTrain,yTrain,layers,@(Y,T) lossFcn(Y,T,trainParams,sysParams),options);
end

function loss = lossFcn(Y,T,trainParams,sysParams)
    AEd  = abs(Y-T);  % vector containing the Squared Error xor each observation
    dataLoss = mean(AEd,"all")/mean(abs(T - mean(T)),"all");
    % physics loss
    % sysParams = params_system();
    f = physics_law(Y,sysParams);
    fTarget = physics_law(T,sysParams);
    disp(f)
    disp(fTarget)
    % physicLoss = mape(f, fTarget,"all");
    AEp  = abs(f-fTarget);  % vector containing the Squared Error xor each observation
    physicLoss = mean(AEp,"all")/mean(abs(fTarget - mean(fTarget)),"all");
    % disp(physicLoss)
    % End Effector loss
    [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(Y(1:5,:),sysParams,"DL");
    [~,~,~,~,xendTarget0,yendTarget0,xendTarget1,yendTarget1,xendTarget2,yendTarget2] = ForwardKinematics(T(1:5,:),sysParams,"DL");
    endEff = [xend0 yend0 xend1 yend1 xend2 yend2];
    endEffTarget = [xendTarget0 yendTarget0 xendTarget1 yendTarget1 xendTarget2 yendTarget2];
    disp(size(endEff))
    disp(size(endEffTarget))
    % endEffloss = mape(endEff,endEffTarget,"all");
    AEe  = abs(endEff-endEffTarget);  % vector containing the Squared Error xor each observation
    endEffloss = mean(AEe,"all")/mean(abs(endEffTarget - mean(endEffTarget)),"all");
    % disp(endEffloss)
    % final loss, combining data loss and physics loss
    % trainParams = params_training();
    loss = (1.0-trainParams.alpha-trainParams.beta)*dataLoss + trainParams.alpha*physicLoss + trainParams.beta*endEffloss;
end
