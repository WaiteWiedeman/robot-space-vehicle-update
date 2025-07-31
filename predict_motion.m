function [xp, tPred] = predict_motion(net, type, t, x, obj, predInterval, seqSteps, initTime)
    numTime = length(t);
    initIdx = find(t > initTime, 1, 'first'); % start where force stop acting
    switch type
        % case "dnn"
        %     xp = zeros(numTime, 15);
        %     xp(1:initIdx, :) = x(1:initIdx, :);
        %     x0 = xp(initIdx, :);
        %     t0 = t(initIdx);
        %     for i = initIdx+1 : numTime
        %         if (t(i)-t0) > predInterval
        %             t0 = t(i-1);
        %             x0 = xp(i-1, :);
        %         end
        %         xp(i,:) = predict(net, [x0, t(i)-t0]);
        %     end
        case {"dnnv2","pgnn"}
            xp = zeros(numTime, 15);
            xp(1:initIdx, :) = x(1:initIdx, :);
            x0 = xp(initIdx, :);
            obj0 = obj(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0) > predInterval
                    t0 = t(i-1);
                    obj0 = obj(i-1, :);
                    x0 = xp(i-1, :);
                end
                tic
                xp(i,:) = predict(net, [x0, obj0, t(i)-t0]);
                tEnds(i) = toc;
            end
        case {"dnn","dnnv2_10s"}
            xp = zeros(numTime, 10);
            xp(1:initIdx, :) = x(1:initIdx, 1:10);
            x0 = xp(initIdx, :);
            obj0 = obj(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0) > predInterval
                    t0 = t(i-1);
                    obj0 = obj(i-1, :);
                    x0 = xp(i-1, :);
                end
                tic
                xp(i,:) = predict(net, [x0, obj0, t(i)-t0]);
                tEnds(i) = toc;
            end
        case "pinn"
            xp = zeros(numTime, 10);
            xp(1:initIdx, :) = x(1:initIdx, 1:10);
            x0 = xp(initIdx, :);
            obj0 = obj(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0 > predInterval)
                    t0 = t(i-1);
                    obj0 = obj(i-1, :);
                    x0 = xp(i-1, :);
                end
                tic
                xp(i,:) = extractdata(predict(net, dlarray([x0, obj0, t(i)-t0]', 'CB')));
                tEnds(i) = toc;
            end
        case "lstm9"
            xp = zeros(numTime, 9);
            xp(1:initIdx, :) = x(1:initIdx, :);
            startIdx = initIdx-seqSteps+1;
            x0 = {[t(startIdx:initIdx), xp(startIdx:initIdx,:)]'};
            t0 = t(initIdx);
            for i = initIdx+1 : numTime          
                if (t(i)-t0) >= predInterval
                    initIdx = i-1;
                    startIdx = initIdx-seqSteps+1;
                    x0 = {[t(startIdx:initIdx), xp(startIdx:initIdx,:)]'};
                    t0 = t(initIdx);
                end
                dsState = arrayDatastore(x0, 'OutputType', 'same', 'ReadSize',1);
                dsTime = arrayDatastore(t(i)-t0, 'ReadSize', 1);
                dsTest = combine(dsState, dsTime);
                xp(i,:) = predict(net, dsTest);
            end
        case {"pinn9", "pirn9"}
            xp = zeros(numTime, 9);
            xp(1:initIdx, :) = x(1:initIdx, :);
            x0 = xp(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0 > predInterval)
                    t0 = t(i-1);
                    x0 = xp(i-1, :);
                end
                xp(i,:) = extractdata(predict(net, dlarray([x0, t(i)-t0]', 'CB')));
            end
        case "dnn6"
            xp = zeros(numTime, 6);
            xp(1:initIdx, :) = x(1:initIdx, 1:6);
            x0 = xp(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0) > predInterval
                    t0 = t(i-1);
                    x0 = xp(i-1, 1:6);
                end
                xp(i,1:6) = predict(net, [x0, t(i)-t0]);
            end
        case "lstm6"
            xp = zeros(numTime, 6);
            xp(1:initIdx, :) = x(1:initIdx, 1:6);
            startIdx = initIdx-seqSteps+1;
            x0 = {[t(startIdx:initIdx), xp(startIdx:initIdx,1:4)]'};
            t0 = t(initIdx);
            for i = initIdx+1 : numTime          
                if (t(i)-t0) >= predInterval
                    initIdx = i-1;
                    startIdx = initIdx-seqSteps+1;
                    x0 = {[t(startIdx:initIdx), xp(startIdx:initIdx,1:6)]'};
                    t0 = t(initIdx);
                end
                dsState = arrayDatastore(x0, 'OutputType', 'same', 'ReadSize',1);
                dsTime = arrayDatastore(t(i)-t0, 'ReadSize', 1);
                dsTest = combine(dsState, dsTime);
                xp(i,1:6) = predict(net, dsTest);
            end
        case {"pinn6", "pirn6"}
            xp = zeros(numTime, 6);
            xp(1:initIdx, :) = x(1:initIdx, 1:6);
            x0 = xp(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0 > predInterval)
                    t0 = t(i-1);
                    x0 = xp(i-1, 1:6);
                end
                xp(i,1:6) = extractdata(predict(net, dlarray([x0, t(i)-t0]', 'CB')));
            end
        case "dnn3"
            xp = zeros(numTime, 3);
            xp(1:initIdx, :) = x(1:initIdx, 1:3);
            x0 = xp(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0) > predInterval
                    t0 = t(i-1);
                    x0 = xp(i-1, 1:3);
                end
                xp(i,1:3) = predict(net, [x0, t(i)-t0]);
            end
        case "lstm2"
            xp = zeros(numTime, 3);
            xp(1:initIdx, :) = x(1:initIdx, 1:3);
            startIdx = initIdx-seqSteps+1;
            x0 = {[t(startIdx:initIdx), xp(startIdx:initIdx,1:3)]'};
            t0 = t(initIdx);
            for i = initIdx+1 : numTime          
                if (t(i)-t0) >= predInterval
                    initIdx = i-1;
                    startIdx = initIdx-seqSteps+1;
                    x0 = {[t(startIdx:initIdx), xp(startIdx:initIdx,1:3)]'};
                    t0 = t(initIdx);
                end
                dsState = arrayDatastore(x0, 'OutputType', 'same', 'ReadSize',1);
                dsTime = arrayDatastore(t(i)-t0, 'ReadSize', 1);
                dsTest = combine(dsState, dsTime);
                xp(i,1:3) = predict(net, dsTest);
            end
        case {"pinn3", "pirn3"}
            xp = zeros(numTime, 3);
            xp(1:initIdx, :) = x(1:initIdx, 1:3);
            x0 = xp(initIdx, :);
            t0 = t(initIdx);
            for i = initIdx+1 : numTime
                if (t(i)-t0 > predInterval)
                    t0 = t(i-1);
                    x0 = x(i-1, 1:3);
                end
                xp(i,1:3) = extractdata(predict(net, dlarray([x0, t(i)-t0]', 'CB')));
            end
        otherwise
            disp("unspecified type of model");
    end
    tPred = mean(tEnds,"all");
end