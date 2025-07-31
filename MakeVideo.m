function MakeVideo(ctrlParams,sysParams, t, x, xp, ref, tSpan)
     % Set up video
    v=VideoWriter('robot_animation.avi');
    v.FrameRate=30;
    open(v);

    idx1 = find(t <= tSpan(1), 1, 'last');
    idx2 = find(t <= tSpan(2), 1, 'last');

    % Animation
    % plot limits
    % Xmin = -7;
    % Xmax = 7;
    % Ymin = -3;
    % Ymax = 3;
   
    f = figure('Color', 'White');
    f.Position = [500 100 900 900];

    for n = idx1:idx2
        cla
        Xcg = x(n,1);
        Ycg = x(n,2);
        alph = x(n,3);
        
        hold on
        % Plot one frame...
        [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(x(n,1:5),sysParams);
        [h1,h3]=robot_plot_frame(ctrlParams,sysParams,t(n),Ycg,Xcg,alph,xend0,yend0,xend1,yend1,xend2,yend2,ref(n,:));
        
        if sum(xp) ~= 0
            Xcg_pred = xp(n,1);
            Ycg_pred = xp(n,2);
            alph_pred = xp(n,3);
            [~,~,~,~,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(xp(n,1:5),sysParams);
            h4=robot_plot_frame_prediction(sysParams,Ycg_pred,Xcg_pred,alph_pred,xend0,yend0,xend1,yend1,xend2,yend2);
        end
        % disErr = Xcg_pred - Xcg;
        % angErr1 = x(n,2) - xp(n,2);
        % angErr2 = x(n,3) - xp(n,3);
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

        tObj = title("System at "+num2str(t(n))+" second", "FontName", "Arial","FontSize",15);
        tObj.Position(1) = -3.0;
        if sum(xp) ~= 0
            legend([h1 h3 h4], "FontName","Arial", "FontSize", 15, 'Location', 'best');
        else
            legend([h1 h3], "FontName","Arial", "FontSize", 15, 'Location', 'best');
        end
        frame=getframe(gcf);
        writeVideo(v,frame);
    end
end

% [~,~,~,~,xend1,yend1,xend2,yend2] = ForwardKinematics(x(n,1:3),sysParams);
        % [~,~,~,~,xpend1,ypend1,xpend2,ypend2] = ForwardKinematics(xp(n,1:3),sysParams);
        % 
        % subplot(3,2,1)
        % plot(t(idx1:n)-1,x(idx1:n,1),'k-',t(idx1:n)-1,xp(idx1:n,1),'r--','LineWidth',2);
        % set(gca, 'FontSize', 12); % Set font size of ticks
        % ylabel('$\theta_0$',"Interpreter","latex"); %, 'FontSize', 18);
        % set(get(gca,'ylabel'),'rotation',0);
        % set(gca, 'FontName', "Arial")
        % axis([0,max(t)-1 min(xp(:,1))-1 max(xp(:,1))+1])
        % set(gca,'Position',[0.07,0.8,0.4,0.15]);
        % xlabel("Time (s)"); %,'FontSize', 15);
        % grid on
        % 
        % subplot(3,2,2)
        % plot(t(idx1:n)-1,x(idx1:n,2),'k-',t(idx1:n)-1,xp(idx1:n,2),'r--','LineWidth',2);
        % set(gca, 'FontSize', 12); % Set font size of ticks
        % ylabel('$\theta_1$',"Interpreter","latex"); %, 'FontSize', 18);
        % set(get(gca,'ylabel'),'rotation',0);
        % set(gca, 'FontName', "Arial")
        % axis([0,max(t)-1 min(xp(:,2))-1 max(xp(:,2))+1])
        % set(gca,'Position',[0.55,0.8,0.4,0.15]);
        % xlabel("Time (s)"); %,'FontSize', 15);
        % grid on;
        % 
        % subplot(3,2,3)
        % plot(t(idx1:n)-1,x(idx1:n,3),'k-',t(idx1:n)-1,xp(idx1:n,3),'r--','LineWidth',2);
        % set(gca, 'FontSize', 12); % Set font size of ticks
        % ylabel('$\theta_2$',"Interpreter","latex"); %, 'FontSize', 18);
        % set(get(gca,'ylabel'),'rotation',0);
        % set(gca, 'FontName', "Arial")
        % axis([0,max(t)-1 min(xp(:,3))-1 max(xp(:,3))+1])
        % set(gca,'Position',[0.07,0.58,0.4,0.15]);
        % xlabel("Time (s)"); %,'FontSize', 15);
        % grid on;
        % 
        % subplot(3,2,4)
        % plot(endeff(idx1:n,1),endeff(idx1:n,2),'b-',endeffp(idx1:n,1),endeffp(idx1:n,2),'r--','LineWidth',2);
        % set(gca, 'FontSize', 12); % Set font size of ticks
        % ylabel("Y"); %, 'FontSize', 15);
        % xlabel("X"); %, 'FontSize', 15);
        % set(get(gca,'ylabel'),'rotation',0);
        % set(gca, 'FontName', "Arial")
        % axis padded
        % set(gca,'Position',[0.55,0.58,0.4,0.15]);
        % grid on;
        % 
        % subplot(3,2,[5,6])
        % hold on
        % % Plot one frame...
        % [h1,h2,h3,h4] = robot_plot_frame(ctrlParams,Xcg_pred,Ycg,cartHalfLen,Xcg,xend1,yend1,xend2,yend2,xpend1,ypend1,xpend2,ypend2,ref);
        % 
        % disErr = Xcg_pred - Xcg;
        % angErr1 = x(n,2) - xp(n,2);
        % angErr2 = x(n,3) - xp(n,3);
        % annotation('textbox', [0.8, 0.27, 0.12, 0.1], ...
        %     'String', {"$\theta_0$ error: "+num2str(disErr,'%.3f') + " m" , "$\theta_1$  error: " + num2str(angErr1,'%.3f') + " rad", "$\theta_2$ error: " + num2str(angErr2,'%.3f') + " rad"}, ...
        %     'FitBoxToText', 'on', ...
        %     'BackgroundColor', 'white', ...
        %     'EdgeColor', 'White', ...
        %     'FontName', 'Arial', ...
        %     'FontSize', 12, ...
        %     "Interpreter","latex");
        % 
        % axis padded %([Xmin Xmax Ymin Ymax])
        % set(gca, "FontName", "Arial");
        % set(gca, "FontSize", 12);
        % xlabel("(m)", "FontSize", 15, "FontName","Arial")
        % ylabel("(m)", "FontSize", 15, "FontName","Arial")
        % daspect([1 1 1])
        % set(gca,'Position',[0.1,0.1,0.8,0.4]);
        % 
        % tObj = title("System at "+num2str(t(n)-1)+" second", "FontName", "Arial","FontSize",15);
        % tObj.Position(1) = -3.0;
        % legend([h1 h2 h3,h4], "FontName","Arial", "FontSize", 12, 'Position', [0.84, 0.4, 0.04, 0.05]);
