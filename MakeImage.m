function MakeImage(ctrlParams,sysParams, t, x, xp, ref, tSpan)
    idx = find(t <= tSpan(2), 1, 'last');

    % Reference
    
    % Animation
    % Ycg = 0;
    % plot limits
    % Xmin = -7;
    % Xmax = 7;
    % Ymin = -3;
    % Ymax = 3;
    % cartHalfLen = 0.4;
    
    f = figure('Color', 'White');
    f.Position = [500 100 900 900]; %[500 200 800 500];
    hold on
    % Plot one frame...
    Xcg = x(idx,1);
    Ycg = x(idx,2);
    alph = x(idx,3);
    [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(x(idx,1:5),sysParams,"normal");
    [h1,h3]=robot_plot_frame(ctrlParams,sysParams,t(idx),Ycg,Xcg,alph,xend0,yend0,xend1,yend1,xend2,yend2,ref(idx,:));
    
    if sum(xp) ~= 0  
        Xcg_pred = xp(idx,1);
        Ycg_pred = xp(idx,2);
        alph_pred = xp(idx,3);
        [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(xp(idx,1:5),sysParams,"normal");
        h4=robot_plot_frame_prediction(sysParams,Ycg_pred,Xcg_pred,alph_pred,xend0,yend0,xend1,yend1,xend2,yend2);
    end
    % disErr = Xcg_pred - Xcg;
    % angErr1 = x(idx,2) - xp(idx,2);
    % angErr2 = x(idx,3) - xp(idx,3);
    % annotation('textbox', [0.8, 0.53, 0.12, 0.1], ...
    %     'String', {"$\theta_0$ error: "+num2str(disErr,'%.3f') + " m" , "$\theta_1$  error: " + num2str(angErr1,'%.3f') + " rad", "$\theta_2$ error: " + num2str(angErr2,'%.3f') + " rad"}, ...
    %     'FitBoxToText', 'on', ...
    %     'BackgroundColor', 'white', ...
    %     'EdgeColor', 'White', ...
    %     'FontName', 'Arial', ...
    %     'FontSize', 11, ...
    %     "Interpreter","latex");
    
    axis padded %([Xmin Xmax Ymin Ymax])
    set(gca, "FontName", "Arial");
    set(gca, "FontSize", 12);
    xlabel("(m)", "FontSize", 15, "FontName","Arial")
    ylabel("(m)", "FontSize", 15, "FontName","Arial")
    daspect([1 1 1])

    % tObj = title("System at "+num2str(tSpan(2)-tSpan(1))+" second", "FontName", "Arial","FontSize",15);
    % tObj.Position(1) = -3.0;
    if sum(xp) ~= 0
        legend([h1 h3 h4], "FontName","Arial", "FontSize", 15, 'Location', 'best');
    else
        legend([h1 h3], "FontName","Arial", "FontSize", 15, 'Location', 'best');
    end
    % saveas(f,'robot_image.jpg')
end
