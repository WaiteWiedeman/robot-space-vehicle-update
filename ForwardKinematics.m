function [x1,y1,x2,y2,xend0,yend0,xend1,yend1,xend2,yend2] = ForwardKinematics(x,sysParams,flag)
    L1 = sysParams.L1;
    L2 = sysParams.L2;
    l1 = L1/2;
    l2 = L2/2;
    b1 = sysParams.b1;
    
    switch flag
        case "normal"
            xv = x(:,1);
            yv = x(:,2);
            alv = x(:,3);
            th1 = x(:,4);
            th2 = x(:,5);
        case "DL"
            xv = x(1,:);
            yv = x(2,:);
            alv = x(3,:);
            th1 = x(4,:);
            th2 = x(5,:);
    end

    x1 = xv + b1/2*cos(alv) + l1*cos(alv+th1);
    y1 = yv + b1/2*sin(alv) + l1*sin(alv+th1);
    x2 = xv + b1/2*cos(alv) + L1*cos(alv+th1) + l2*cos(alv+th1+th2);
    y2 = yv + b1/2*sin(alv) + L1*sin(alv+th1) + l2*sin(alv+th1+th2);
    xend0 = xv + b1/2*cos(alv);
    yend0 = yv + b1/2*sin(alv);
    xend1 = xv + b1/2*cos(alv) + L1*cos(alv+th1);
    yend1 = yv + b1/2*sin(alv) + L1*sin(alv+th1);
    xend2 = xv + b1/2*cos(alv) + L1*cos(alv+th1) + L2*cos(alv+th1+th2);
    yend2 = yv + b1/2*sin(alv) + L1*sin(alv+th1) + L2*sin(alv+th1+th2);
end
