function [xTrain,yTrain,layers,options] = train_dnnv2_model_10state(sampleFile, trainParams)
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
        x = data(2:11,:);
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
    numStates = 10;
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
        fullyConnectedLayer(numStates, "Name", "output")
        DNNLossLayer("mse")]; % weightedLossLayer("mse") regressionLayer

    layers = layerGraph(layers);
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
end
    