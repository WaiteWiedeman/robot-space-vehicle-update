clc; clear; close all;

%% params
ctrlParams = params_control();
sysParams = params_system(); 
trainParams = params_training();

%% random points
t=0:0.01:5;
x = ctrlParams.a*cos((2*pi/5)*t);
y = ctrlParams.b*sin((2*pi/5)*t);

endx = 3 + 2*cos((2*pi/5)*t);
endy = 2*sin((2*pi/5)*t);

for i = 1:100
    theta = 2*pi*rand;
    rad = sqrt(rand);
    ctrlParams.refx = ctrlParams.a*rad*cos(theta);
    ctrlParams.refy = ctrlParams.b*rad*sin(theta);
    Xd(i) = ctrlParams.refx;
    Yd(i) = ctrlParams.refy;
end

plot(x,y,'k-',endx,endy,'b-',Xd,Yd,'*')

%% IK check
theta = 2*pi*rand;
rad = sqrt(rand);
ctrlParams.refx = -4; %ctrlParams.a*rad*cos(theta);
ctrlParams.refy = ctrlParams.b*rad*sin(theta);
[Xd, Yd, Xdd, Ydd, Xc, Xcd] = referenceTrajectory(0,ctrlParams);
[Th1,Th2,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xc,Xcd);
[x1,y1,x2,y2,xend1,yend1,xend2,yend2] = ForwardKinematics([Xc Th1 Th2],sysParams);

%% Cell array manipulation
% a = 1:200;
% dataSize = length(a);
% nmPts = 64;
% nmGrps = ceil(dataSize/nmPts);
% b = cell(nmGrps,1);
% for i = 1:nmGrps
%     startIdx = (i-1)*nmPts + 1;
%     endIdx = min(i*nmPts, dataSize);
%     b{i} = a(startIdx:endIdx);
% end    

sampleFile = "trainingSamples.mat";
ds = load(sampleFile);
numSamples = 100;

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

miniBatchSize = trainParams.miniBatchSize/trainParams.nmPts;
dataSize = length(xTrain);
numBatches = ceil(dataSize/miniBatchSize);
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

shuffle(mbq)
[X,T] = next(mbq);

[loss] = modelLoss(X, T, sysParams, trainParams);
disp(loss)
% xBatch = {};
% tBatch = {};
% X = extractdata(X);
% T = extractdata(T);
% 
% ids = find(diff(X(1,:)) ~= 0);
% sz = size(X);
% % startIds = [ids+1];
% boundaryIds = [ids+1 ids sz(2)];
% 
% q1d = gradient(T(6,:),X(20,:)); %zeros(1,sz(2));
% q1d2 = 4*del2(T(1,:),X(20,:));
% 
% q1dtrue = T(11,:);
% 
% q1d(boundaryIds) = [];
% q1d2(boundaryIds) = [];
% q1dtrue(boundaryIds) = [];
% 
% err1 = mean(q1dtrue-q1d);
% err2 = mean(q1dtrue-q1d2);
% err3 = mean(q1d-q1d2);
% disp(err1)
% disp(err2)
% disp(err3)

%% PGNN check
sampleFile = "trainingSamples.mat";
ds = load(sampleFile);
numSamples = 1; %trainParams.numSamples;

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
loss = lossFcn(yTrain(:,1),yTrain(:,1),trainParams,sysParams);
disp(loss)

%% functions
function [loss] = modelLoss(X, T, sysParams, trainParams)
    % Get parameters
    % sysParams = params_system();
    % trainParams = params_training();

    % Split inputs and targets into cell arrays

    % [Z, ~] = forward(net, X);
    Z = T;
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
    disp("yo 1")
    [fY,fT,endEff,endEffTarget] = physicsloss(X(15,:),T,Z,boundaryIds,sysParams);
    disp("yo 2")
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
    disp(dataLoss)
    disp(physicLoss)
    disp(endEffloss)
    loss = (1.0-trainParams.alpha-trainParams.beta)*dataLoss + trainParams.alpha*physicLoss + trainParams.beta*endEffloss;
    % loss = dataLoss;
    % gradients = dlgradient(loss, net.Learnables);
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
    % q1dn = rdiff_kalman(q1,T,[], 'ncp');
    % q2dn = rdiff_kalman(q2,T,[], 'ncp');
    % q3dn = rdiff_kalman(q3,T,[], 'ncp');
    % q4dn = rdiff_kalman(q4,T,[], 'ncp');
    % q5dn = rdiff_kalman(q5,T,[], 'ncp');
    % accelerations calulated as gradient of velocity prediction
    q1ddn = gradient(q1d,T);
    q2ddn = gradient(q2d,T);
    q3ddn = gradient(q3d,T);
    q4ddn = gradient(q4d,T);
    q5ddn = gradient(q5d,T);
    % q1ddn = rdiff_kalman(q1d,T,[], 'ncp');
    % q2ddn = rdiff_kalman(q2d,T,[], 'ncp');
    % q3ddn = rdiff_kalman(q3d,T,[], 'ncp');
    % q4ddn = rdiff_kalman(q4d,T,[], 'ncp');
    % q5ddn = rdiff_kalman(q5d,T,[], 'ncp');
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
    % q1ddnT = rdiff_kalman(Y(6,:),T,[], 'ncp');
    % q2ddnT = rdiff_kalman(Y(7,:),T,[], 'ncp');
    % q3ddnT = rdiff_kalman(Y(8,:),T,[], 'ncp');
    % q4ddnT = rdiff_kalman(Y(9,:),T,[], 'ncp');
    % q5ddnT = rdiff_kalman(Y(10,:),T,[], 'ncp');
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
    [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(transpose(Z(1:5,:)),sysParams);
    [~,~,~,~,xendTarget0,yendTarget0,xendTarget1,yendTarget1,xendTarget2,yendTarget2] = ForwardKinematics(transpose(Y(1:5,:)),sysParams);

    endEff = [xend0 yend0 xend1 yend1 xend2 yend2];
    endEffTarget = [xendTarget0 yendTarget0 xendTarget1 yendTarget1 xendTarget2 yendTarget2];
end

function [X,T] = myMiniBatch(xBatch,yBatch)
    X = [];
    T = [];
    for i = 1:length(xBatch)
        X = [X, cell2mat(xBatch{i})];
        T = [T, cell2mat(yBatch{i})];
    end
    % disp(xBatch)
    % X = cat(2,xBatch{:});
    % disp(X)
    % T = cat(2,yBatch{:});
end

function loss = lossFcn(Y,T,trainParams,sysParams)
    AEd  = abs(Y-T);  % vector containing the Squared Error xor each observation
    dataLoss = mean(AEd,"all")/mean(abs(T - mean(T)),"all");
    % physics loss
    % sysParams = params_system();
    f = physics_law(Y,sysParams);
    fTarget = physics_law(T,sysParams);
    % physicLoss = mape(f, fTarget,"all");
    AEp  = abs(f-fTarget);  % vector containing the Squared Error xor each observation
    physicLoss = mean(AEp,"all")/mean(abs(fTarget - mean(fTarget)),"all");
    % disp(physicLoss)
    % End Effector loss
    [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(Y(1:5)',sysParams);
    [~,~,~,~,xendTarget0,yendTarget0,xendTarget1,yendTarget1,xendTarget2,yendTarget2] = ForwardKinematics(T(1:5)',sysParams);
    endEff = [xend0;yend0;xend1;yend1;xend2;yend2];
    endEffTarget = [xendTarget0;yendTarget0;xendTarget1;yendTarget1;xendTarget2;yendTarget2];
    % endEffloss = mape(endEff,endEffTarget,"all");
    AEe  = abs(endEff-endEffTarget);  % vector containing the Squared Error xor each observation
    endEffloss = mean(AEe,"all")/mean(abs(endEffTarget - mean(endEffTarget)),"all");
    % disp(endEffloss)
    % final loss, combining data loss and physics loss
    % trainParams = params_training();
    loss = (1.0-trainParams.alpha-trainParams.beta)*dataLoss + trainParams.alpha*physicLoss + trainParams.beta*endEffloss;
end