function plot_compared_states(t,x,tp,xp,flag,refs)
Idx = find(t >= 1, 1, 'first');
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
            plot(tp(Idx:end),xp(Idx:end,i),'r--','LineWidth',2);
            % xline(1,'k--', 'LineWidth',1);
            if i == 1 || i ==2
                ylabel(labels(i) + "[m]","Interpreter","latex");
            else
                ylabel(labels(i) + "[rad]","Interpreter","latex");
            end
            % set(get(gca,'ylabel'),'rotation',0);
            grid on
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-10
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Reference Trajectory","Prediction","Location","eastoutside","FontName","Arial");
    case "velocity"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 6:numState-5
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            plot(tp(Idx:end),xp(Idx:end,i),'r--','LineWidth',2);
            % xline(1,'k--', 'LineWidth',1);
            if i == 6 || i == 7
                ylabel(labels(i) + "[m/s]","Interpreter","latex");
            else
                ylabel(labels(i) + "[rad/s]","Interpreter","latex");
            end
            % set(get(gca,'ylabel'),'rotation',0);
            grid on
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState-3
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
    case "acceleration"
        figure('Position',[500,200,800,800]);
        tiledlayout("vertical","TileSpacing","tight")
        numState = size(x);
        numState = numState(2);
        for i = 11:numState
            nexttile
            plot(t,x(:,i),'b-','LineWidth',2);
            hold on
            plot(tp(Idx:end),xp(Idx:end,i),'r--','LineWidth',2);
            % xline(1,'k--', 'LineWidth',1);
            if i == 11 || i == 12
                ylabel(labels(i) + "[m/s/s]","Interpreter","latex");
            else
                ylabel(labels(i) + "[rad/s/s]","Interpreter","latex");
            end
            set(get(gca,'ylabel'),'rotation',0);
            grid on
            set(gca, 'FontSize', 15);
            set(gca, 'FontName', "Arial")
            if i == numState
                xlabel("Time (s)");
            end
        end
        legend("Ground Truth","Prediction","Location","eastoutside","FontName","Arial");
end
end