function dataFile = generate_samples(sysParams, ctrlParams, trainParams)
% Generate samples and save the data file into a subfolder "data\"
    tSpan = [0,20];
    dataFile = "trainingSamples.mat";
    % check whether need to regenerate samples
    regenerate_samples = 1; % by default, regrenerate samples
    if exist(dataFile, 'file') == 2
        ds = load(dataFile);
        if trainParams.numSamples == length(ds.samples)
            regenerate_samples = 0;
        end
    end
    
    % generate sample data
    if regenerate_samples      
        samples = {};
        for i = 1:trainParams.numSamples
            disp("generate data for " + num2str(i) + "th sample.");
            theta = 2*pi*rand;
            rad = sqrt(rand);
            ctrlParams.refx = ctrlParams.xrange*rad*cos(theta);
            ctrlParams.refy = ctrlParams.yrange*rad*sin(theta);
            ctrlParams.phi = 2*pi*rand;
            ctrlParams.a = 0.25+rand*0.25; % target object horizontal dimension
            ctrlParams.b = 0.25+rand*0.25; % vertical dimension
            switch trainParams.datasource
                case "odes"
                    x0 = [-1; -1; 0; 0; 0] + [2; 2; 2*pi; 2*pi; 2*pi].*rand(5,1); % th0, th1, th2
                    x0 = [x0(1); 0; x0(2); 0; x0(3); 0; x0(4); 0; x0(5); 0]; % th0, th0d, th1, th1d, th2, th2d
                    y = robot_simulation(tSpan, x0, sysParams, ctrlParams);
                case "simscape"
                    mdl = "robot_model.slx";
                    assignin('base','ctrlParams',ctrlParams)
                    y = run_simscape(mdl,ctrlParams);
            end
            state = y';
            fname=['data\input',num2str(i),'.mat'];
            save(fname, 'state');
            samples{end+1} = fname;
        end
        samples = reshape(samples, [], 1); % make it row-based
        save(dataFile, 'samples');
    else
        %disp(num2str(trainParams.numSamples) + " samples is already generated.");
    end
end