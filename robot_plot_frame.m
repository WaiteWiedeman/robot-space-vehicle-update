function [h1,h3]=robot_plot_frame(ctrlParams,sysParams,t,Ycg,Xcg,alph,xend0,yend0,xend1,yend1,xend2,yend2,ref)
    % True system
    b1 = sysParams.b1/2;
    b2 = sysParams.b2/2;
    r = sqrt(b1^2+b2^2);
    phi1 = atan2(b2,b1);
    phi2 = atan2(b2,-b1);
    phi3 = atan2(-b2,-b1);
    phi4 = atan2(-b2,b1);
    
    % plots reference being tracked by arm
    Xd = ref(1);
    Yd = ref(2);
    h3 = plot(Xd,Yd,'Marker','o','MarkerSize',14,'MarkerEdgeColor','b', "DisplayName", "Objective");
    X_ellipse = ctrlParams.refx + ctrlParams.a*cos(0:0.01:2*pi)*cos((2*pi/5)*t) - ...
        ctrlParams.b*sin(0:0.01:2*pi)*sin((2*pi/5)*t); 
    Y_ellispe = ctrlParams.refy + ctrlParams.b*sin(0:0.01:2*pi)*cos((2*pi/5)*t) + ...
        ctrlParams.a*cos(0:0.01:2*pi)*sin((2*pi/5)*t); 
    patch(X_ellipse,Y_ellispe,'g','LineWidth', 2); %, 'LineStyle',"-."

    % cart
    patch(Xcg+[r*cos(phi1+alph) r*cos(phi2+alph) r*cos(phi3+alph) r*cos(phi4+alph)], ...
        Ycg+[r*sin(phi1+alph) r*sin(phi2+alph) r*sin(phi3+alph) r*sin(phi4+alph)],'k','FaceColor','#808080','LineWidth',2); %,'FaceAlpha', 0 ,'k' 'FaceColor','#808080',
    % plots rod and blob
    h1 = plot([xend0 xend1],[yend0 yend1],'k','LineWidth', 3, 'LineStyle','-', "DisplayName", "Ground Truth");
    plot(xend0,yend0,'Marker','o','MarkerSize',12,'MarkerEdgeColor','k');
    plot(xend1,yend1,'Marker','o','MarkerSize',12,'MarkerEdgeColor','k');
    plot([xend1 xend2],[yend1 yend2],'k','LineWidth', 3, 'LineStyle','-');
    plot(xend2,yend2,'Marker','o','MarkerSize',12,'MarkerEdgeColor','k');
    
    % System predicted by model
    % cart
    % patch('XData', Xcg_pred+[-cartHalfLen cartHalfLen cartHalfLen -cartHalfLen],...
    %     'YData', Ycg+[cartHalfLen cartHalfLen -cartHalfLen -cartHalfLen],...
    %     'FaceColor','none', 'FaceAlpha', 0, ...
    %     'EdgeColor','r','LineWidth',2,'LineStyle','--');
    % 
    % % plots pendulum
    % h2 = plot([Xcg_pred xpend1],[Ycg ypend1],'r','LineWidth', 2, 'LineStyle','--', "DisplayName", "Prediction"); 
    % plot(xpend1,ypend1,'Marker','o','MarkerSize',12,'MarkerEdgeColor','r'); 
    % plot([xpend1 xpend2],[ypend1 ypend2],'r','LineWidth', 2, 'LineStyle','--'); 
    % plot(xpend2,ypend2,'Marker','o','MarkerSize',12,'MarkerEdgeColor','r');

    

    % plots boundary of random objective points
    % boundx = ctrlParams.xrange*cos(0:0.01:2*pi);
    % boundy = ctrlParams.yrange*sin(0:0.01:2*pi);
    % h4 = plot(boundx,boundy,'k','LineWidth', 2, 'LineStyle',':', "DisplayName", "Objective bound");
end
