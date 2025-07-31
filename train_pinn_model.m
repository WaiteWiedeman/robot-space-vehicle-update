function output = train_pinn_model(sampleFile, trainParams,sysParams,ctrlParams,monitor)
% PINN
% A physics-Informed Neural Network (PINN) is a type of neural network
% architecture desigend to incorporate physical principles or equations
% into the learning process. In combines deep learning techniques with
% domain-specific knowledge, making it particularly suitable for problems
% governed by physics.
% In addition to standard data-driven training, PINNs utilize terms in the
% loss function to enforce consistency with know physical law, equations,
% and constraints. 
% https://en.wikipedia.org/wiki/Physics-informed_neural_networks 
% https://benmoseley.blog/my-research/so-what-is-a-physics-informed-neural-network/
    
    %initialize output
    output.trainedNet = [];

    % load samples and prepare training dataset
    ds = load(sampleFile);
    numSamples = trainParams.numSamples;   
    
    % generate data
    % Feature data: 6-D initial state x0 + time interval
    % the label data is a predicted state x=[q1,q2,q1dot,q2dot,q1ddot,q2ddot]
    initTimes = 1:trainParams.initTimeStep:6; %start from 1 sec to 4 sec with 0.5 sec step
    % tGroup = [];
    xGroup = [];
    yGroup = [];
    % tTrain = {};
    xTrain = {};
    yTrain = {};
    for i = 1:numSamples
        data = load(ds.samples{i,1}).state;
        t = data(1,:);
        x = data(2:11, :); % q1,q2,q1_dot,q2_dot
        obj = data(22:25,:);
        for tInit = initTimes
            initIdx = find(t > tInit, 1, 'first');
            x0 = x(:, initIdx); % Initial state
            obj0 = obj(:,initIdx);  % Initial state
            t0 = t(initIdx); % Start time
            for j = initIdx+1 : length(t)
                % tGroup = [tGroup, t(j)-t0];
                xGroup = [xGroup, [x0; obj0; t(j)-t0]];
                yGroup = [yGroup, x(:,j)];
            end
            dataSize = length(xGroup);
            nmGrps = ceil(dataSize/trainParams.nmPts);
            for z = 1:nmGrps
                startIdx = (z-1)*trainParams.nmPts + 1;
                endIdx = min(z*trainParams.nmPts, dataSize);
                if length(xGroup(startIdx:endIdx)) < (trainParams.nmPts/5)
                    xTrain(end) = {[cell2mat(xTrain(end)) xGroup(:,startIdx:endIdx)]};
                    yTrain(end) = {[cell2mat(yTrain(end)) yGroup(:,startIdx:endIdx)]};
                else
                    % tTrain = [tTrain tGroup(startIdx:endIdx)];
                    xTrain = [xTrain xGroup(:,startIdx:endIdx)];
                    yTrain = [yTrain yGroup(:,startIdx:endIdx)];
                end
            end
            % tGroup = [];
            xGroup = [];
            yGroup = [];
        end
    end
    disp(num2str(length(cell2mat(xTrain))) + " samples are generated for training.");
    
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
       ];

    % convert the layer array to a dlnetwork object
    net = dlnetwork(layers);
    net = dlupdate(@double, net);
    % plot(net)
    
    % training options
    monitor.Metrics = "Loss";
    monitor.Info = ["LearnRate" ... 
                    "IterationPerEpoch" ...
                    "MaximumIteration" ...
                    "Epoch" ...
                    "Iteration" ...
                    "GradientsNorm"...
                    "StepNorm"...
                    "TestAccuracy"...
                    "ExecutionEnvironment"];
    monitor.XLabel = "Iteration";
    
    % trainParams.alpha = trainParams.alpha^3;
    % trainParams.beta = trainParams.beta*0.01;

    net = train_adam_update(net, xTrain, yTrain, trainParams, sysParams, monitor);
    save("model\temp", 'net','monitor');
    output.trainedNet = net;

    % ctrlParams.solver = "nonstifflr"; % "stiff" or "normal"
    % ctrlParams.method = "origin"; % random, interval, origin
    % tSpan = [0,20];
    % predInterval = tSpan(2);
    % numCase = 20;
    % numTime = 500;
    % initTime = 1;
    % [avgErr,~,~,~] = evaluate_model(net, sysParams, ctrlParams, trainParams, tSpan, predInterval, numCase, numTime, trainParams.type,0, initTime);
    % avgErr = 5;
    updateInfo(monitor,...
        TestAccuracy=0);
end

%%
function net = train_adam_update(net, xTrain, yTrain, trainParams, sysParams, monitor)
    % using stochastic gradient decent
    miniBatchSize = trainParams.miniBatchSize/trainParams.nmPts;
    lrRate = trainParams.initLearningRate;
    dataSize = length(xTrain);
    numBatches = floor(dataSize/miniBatchSize);
    numIterations = trainParams.numEpochs * numBatches;

    dsState = arrayDatastore(xTrain, "ReadSize", miniBatchSize,"IterationDimension",2); %"OutputType", "same", 
    dsLabel = arrayDatastore(yTrain, "ReadSize", miniBatchSize,"IterationDimension",2);
    dsTrain = combine(dsState, dsLabel);

    mbq = minibatchqueue(dsTrain,...
        MiniBatchSize=miniBatchSize, ...
        MiniBatchFormat="CB", ...
        MiniBatchFcn=@myMiniBatch, ...
        OutputEnvironment="gpu", ...
        PartialMiniBatch="discard");

    accFcn = dlaccelerate(@modelLoss);
    
    avgGrad = [];
    avgSqGrad = [];
    iter = 0;
    epoch = 0;
    while epoch < trainParams.numEpochs && ~monitor.Stop
        epoch = epoch + 1;
        % Shuffle data.
        shuffle(mbq);

        while hasdata(mbq) && ~monitor.Stop
            iter = iter + 1;

            % Read mini-batch of data.
            [X,T] = next(mbq);

            % Evaluate the model loss and gradients using dlfeval and the
            % modelLoss function.
            [loss, gradients] = dlfeval(accFcn, net, X, T, sysParams, trainParams);
            
            % Update the network parameters using the ADAM optimizer.
            [net, avgGrad, avgSqGrad] = adamupdate(net, gradients, avgGrad, avgSqGrad, iter, lrRate);
            
            recordMetrics(monitor, iter, Loss=loss);
    
            if mod(iter, trainParams.numEpochs) == 0
                monitor.Progress = 100*iter/numIterations;
                updateInfo(monitor, ...
                    LearnRate = lrRate, ...
                    Epoch = epoch, ...
                    Iteration = iter, ...
                    MaximumIteration = numIterations, ...
                    IterationPerEpoch = numBatches);
            end

            executionEnvironment = "auto";

            if (executionEnvironment == "auto" && canUseGPU) || executionEnvironment == "gpu"
                updateInfo(monitor,ExecutionEnvironment="GPU");
            else
                updateInfo(monitor,ExecutionEnvironment="CPU");
            end
        end
        % adaptive learning rate
        if mod(epoch,trainParams.lrDropEpoch) == 0
            if lrRate > trainParams.stopLearningRate
                lrRate = lrRate*trainParams.lrDropFactor;
            else
                lrRate = trainParams.stopLearningRate;
            end
        end
    end
end

%% loss function
function [loss, gradients] = modelLoss(net, X, T, sysParams, trainParams)
    % Get parameters
    % sysParams = params_system();
    % trainParams = params_training();

    % Split inputs and targets into cell arrays

    [Z, ~] = forward(net, X);
    % dataLoss = mape(Z, T,'all');
    % dataLoss = mse(Z, T);
    AEd  = abs(Z-T);  % vector containing the Squared Error xor each observation
    dataLoss = mean(AEd,"all")/mean(abs(T - mean(T)),"all");  % Mean-Squared Error

    X = extractdata(X);
    T = extractdata(T);
    Z = extractdata(Z);

    ids = find(diff(X(1,:)) ~= 0);
    sz = size(X);
    boundaryIds = [ids+1 ids sz(2)];

    % disp(size(X))
    [fY,fT,endEff,endEffTarget] = physicsloss(X(15,:),T,Z,boundaryIds,sysParams);

    % disp(size(fY))
    % disp(size(fT))
    % disp(size(endEff))
    % disp(size(endEffTarget))
    % convert prediction and target vectors into dlarrays
    forcePreds = dlarray(fY, "CB");
    forceTargets = dlarray(fT, "CB");
    endEffPreds = dlarray(endEff, "CB");
    endEffTargets = dlarray(endEffTarget, "CB");

    % total loss
    % physicLoss = mae(forcePreds, forceTargets,"all");
    AEp  = abs(forcePreds-forceTargets);  % vector containing the Squared Error xor each observation
    physicLoss = mean(AEp,"all")/mean(abs(forceTargets - mean(forceTargets)),"all");  % Mean-Squared Error
    % endEffloss = mape(endEffPreds, endEffTargets,"all");
    AEe  = abs(endEffPreds-endEffTargets);  % vector containing the Squared Error xor each observation
    endEffloss = mean(AEe,"all")/mean(abs(endEffTargets - mean(endEffTargets)),"all");  % Mean-Squared Error
    % disp(dataLoss)
    % disp(physicLoss)
    % disp(endEffloss)
    loss = (1.0-trainParams.alpha-trainParams.beta)*dataLoss + trainParams.alpha*physicLoss + trainParams.beta*endEffloss;
    % loss = dataLoss;
    gradients = dlgradient(loss, net.Learnables);
end

function [fY,fT,endEff,endEffTarget] = physicsloss(T,Y,Z,ids,sysParams)
    % compute gradients using automatic differentiation
    q1 = Z(1,:);
    q2 = Z(2,:);
    q3 = Z(3,:);
    q4 = Z(4,:);
    q5 = Z(5,:);
    q1d = Z(6,:);
    q2d = Z(7,:);
    q3d = Z(8,:);
    q4d = Z(9,:);
    q5d = Z(10,:);

    % velocities calulated as gradient of position prediction
    q1dn = gradient(q1,T);
    q2dn = gradient(q2,T);
    q3dn = gradient(q3,T);
    q4dn = gradient(q4,T);
    q5dn = gradient(q5,T);
    % accelerations calulated as gradient of velocity prediction
    q1ddn = gradient(q1d,T);
    q2ddn = gradient(q2d,T);
    q3ddn = gradient(q3d,T);
    q4ddn = gradient(q4d,T);
    q5ddn = gradient(q5d,T);
    % accelerations calulated as gradient of velocity prediction
    % q1ddn2 = 4*del2(q1,T);
    % q2ddn2 = 4*del2(q2,T);
    % q3ddn2 = 4*del2(q3,T);
    % q4ddn2 = 4*del2(q4,T);
    % q5ddn2 = 4*del2(q5,T);
    % target gradients
    % accelerations calulated as gradient of velocity prediction
    q1ddnT = gradient(Y(6,:),T);
    q2ddnT = gradient(Y(7,:),T);
    q3ddnT = gradient(Y(8,:),T);
    q4ddnT = gradient(Y(9,:),T);
    q5ddnT = gradient(Y(10,:),T);
    % accelerations calulated as gradient of velocity prediction
    % q1ddn2T = 4*del2(Y(1,:),T);
    % q2ddn2T = 4*del2(Y(2,:),T);
    % q3ddn2T = 4*del2(Y(3,:),T);
    % q4ddn2T = 4*del2(Y(4,:),T);
    % q5ddn2T = 4*del2(Y(5,:),T);
% disp(q1ddn)
    % remove values at boundary 
    q1(ids) = [];
    q2(ids) = [];
    q3(ids) = [];
    q4(ids) = [];
    q5(ids) = [];

    q1d(ids) = [];
    q2d(ids) = [];
    q3d(ids) = [];
    q4d(ids) = [];
    q5d(ids) = [];

    q1dn(ids) = [];
    q2dn(ids) = [];
    q3dn(ids) = [];
    q4dn(ids) = [];
    q5dn(ids) = [];

    q1ddn(ids) = [];
    q2ddn(ids) = [];
    q3ddn(ids) = [];
    q4ddn(ids) = [];
    q5ddn(ids) = [];

    % q1ddn2(ids) = [];
    % q2ddn2(ids) = [];
    % q3ddn2(ids) = [];
    % q4ddn2(ids) = [];
    % q5ddn2(ids) = [];

    q1ddnT(ids) = [];
    q2ddnT(ids) = [];
    q3ddnT(ids) = [];
    q4ddnT(ids) = [];
    q5ddnT(ids) = [];

    % q1ddn2T(ids) = [];
    % q2ddn2T(ids) = [];
    % q3ddn2T(ids) = [];
    % q4ddn2T(ids) = [];
    % q5ddn2T(ids) = [];

    Y(:,ids) = [];
    Z(:,ids) = [];

    fY = physics_law([q1;q2;q3;q4;q5;(0.5*q1d+0.5*q1dn);(0.5*q2d+0.5*q2dn);(0.5*q3d+0.5*q3dn);(0.5*q4d+0.5*q4dn);(0.5*q5d+0.5*q5dn);...
        q1ddn;q2ddn;q3ddn;q4ddn;q5ddn],sysParams);
    fT = physics_law([Y;q1ddnT;q2ddnT;q3ddnT;q4ddnT;q5ddnT],sysParams);
% disp(fY)
% disp(fT)
    [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(Z(1:5,:),sysParams,"DL");
    [~,~,~,~,xendTarget0,yendTarget0,xendTarget1,yendTarget1,xendTarget2,yendTarget2] = ForwardKinematics(Y(1:5,:),sysParams,"DL");

    endEff = [xend0; yend0; xend1; yend1; xend2; yend2];
    endEffTarget = [xendTarget0; yendTarget0; xendTarget1; yendTarget1; xendTarget2; yendTarget2];
end

function [X,T] = myMiniBatch(xBatch,yBatch)
    X = [];
    T = [];
    for i = 1:length(xBatch)
        X = [X, cell2mat(xBatch{i})];
        T = [T, cell2mat(yBatch{i})];
    end
end