function xdot = robot_xdot(x, F, T_f, sysParams)
    xv = x(1); % vehicle x position
    xvd = x(2); % vehicle x velocity
    yv = x(3); % vehicle y position
    yvd = x(4); % vehicle y velocity
    alv = x(5); % vehicle pitch angle
    alvd = x(6); % vehicle pitch velocity
    th1 = x(7); % link 1 position
    th1d = x(8); % link 1 velocity
    th2 = x(9); % link 2 position
    th2d = x(10); % link 2 velocity
    
    % system parameters
    b1 = sysParams.b1;
    b2 = sysParams.b2;
    L1 = sysParams.L1;
    L2 = sysParams.L2;
    G = sysParams.G;
    M0 = sysParams.M0;
    M1 = sysParams.M1;
    M2 = sysParams.M2;
    l1 = L1/2;
    l2 = L2/2;
    I0 = (1/12)*M0*(b1^2+b2^2);
    I1 = (1/12)*M1*L1^2;
    I2 = (1/12)*M2*L2^2;

    % solve the Lagrange equation F = M*qdd + C*qd + G
    % compute qdd: M*qdd = F - C*qd - G, using linsolve
    M = zeros(5);
    C = zeros(5);
    M(:,1) = [M0+M1+M2
              0
              -((L1*M2+l1*M1)*sin(alv+th1)+b1/2*(M1+M2)*sin(alv)+l2*M2*sin(alv+th1+th2))
              -((L1*M2+l1*M1)*sin(alv+th1)+l2*M2*sin(alv+th1+th2))
              -l2*M2*sin(alv+th1+th2)];

    M(:,2) = [0
              M0+M1+M2
              ((L1*M2+l1*M1)*cos(alv+th1)+b1/2*(M1+M2)*cos(alv)+l2*M2*cos(alv+th1+th2))
              ((L1*M2+l1*M1)*cos(alv+th1)+l2*M2*cos(alv+th1+th2))
              l2*M2*cos(alv+th1+th2)];

    M(:,3) = [-((M1*l1+L1*M2)*sin(alv+th1)+b1/2*(M1+M2)*sin(alv)+M2*l2*sin(alv+th1+th2))
              ((M1*l1+L1*M2)*cos(alv+th1)+b1/2*(M1+M2)*cos(alv)+M2*l2*cos(alv+th1+th2))
              (I0+l1^2*M1+(l2^2+L1^2)*M2+b1^2/4*(M1+M2)+b1*(L1*M2+l1*M1)*cos(th1)+2*L1*l2*M2*cos(th2)+b1*l2*M2*cos(th1+th2))
              ((l2^2+L1^2)*M2+l1^2*M1+b1/2*l2*M2*cos(th1+th2)+b1/2*(L1*M2+l1*M1)*cos(th1)+2*L1*l2*M2*cos(th2)) 
              l2^2*M2+b1/2*l2*M2*cos(th1+th2)+L1*l2*M2*cos(th2)];

    M(:,4) = [-((M1*l1+L1*M2)*sin(alv+th1)+M2*l2*sin(alv+th1+th2))
              ((M1*l1+L1*M2)*cos(alv+th1)+M2*l2*cos(alv+th1+th2))
              ((l2^2+L1^2)*M2+l1^2*M1+b1/2*l2*M2*cos(th1+th2)+b1/2*(L1*M2*l1*M1)*cos(th1)+2*L1*l2*M2*cos(th2))
              (I1+(l2^2+L1^2)*M2+l1^2*M1+2*L1*l2*M2*cos(th2))
              (l2^2*M2+L1*l2*M2*cos(th2))];
    M(:,5) = [-M2*l2*sin(alv+th1+th2)
              M2*l2*cos(alv+th1+th2)
              (l2^2*M2+b1/2*l2*M2*cos(th1+th2)+L1*l2*M2*cos(th2))
              (l2^2*M2+L1*l2*M2*cos(th2))
              (I2+l2^2*M2)];
    C(:,1:2) = zeros(5,2);
    C(:,3) = [-((M1*l1+M2*L1)*alvd*cos(alv+th1)+b1/2*(M1+M2)*alvd*cos(alv)+M2*l2*alvd*cos(alv+th1+th2))
              -(b1/2*(M1+M2)*alvd*sin(alv)+(M1*l1+M2*L1)*alvd*sin(alv+th1)+M2*l2*alvd*sin(alv+th1+th2))
              0
              (b1/2*l2*M2*alvd*sin(th1+th2)+b1/2*(M1*l1+M2*L1)*alvd*sin(th1))
              (b1/2*l2*M2*alvd*sin(th1+th2)+L1*l2*M2*alvd*sin(th2))];
    C(:,4) = [-((M1*l1+M2*L1)*cos(alv+th1)*(2*alvd+th1d)+M2*l2*cos(alv+th1+th2)*(2*alvd+th1d))
              -((M1*l1+M2*L1)*sin(alv+th1)*(2*alvd+th1d)+M2*l2*sin(alv+th1+th2)*(2*alvd+th1d))
              -(b1/2*l2*M2*th1d*sin(th1+th2)+b1/2*(L1*M2+l1*M1)*th1d*sin(th1)+b1*l2*M2*alvd*sin(th1+th2)...
              +b1*(L1*M2+l1*M1)*alvd*sin(th1))
              0
              (L1*l2*M2*sin(th2)*(2*alvd+th1d))];
    C(:,5) = [-M2*l2*cos(alv+th1+th2)*(2*alvd+2*th1d+th2d)
              -M2*l2*sin(alv+th1+th2)*(2*alvd+2*th1d+th2d)
              -(b1*l2*M2*sin(th1+th2)*(th2d+2*th1d+2*alvd)+L1*l2*M2*sin(th2)*(th2d+2*th1d+2*alvd))
              -L1*l2*M2*sin(th2)*(2*alvd+2*th1d+th2d)
              0];
    Gmat = [0
         (M0+M1+M2)
         ((L1*M2+l1*M1)*cos(alv+th1)+b1/2*(M1+M2)*cos(alv)+l2*M2*cos(alv+th1+th2))
         ((L1*M2+l1*M1)*cos(alv+th1)+l2*M2*cos(alv+th1+th2))
         l2*M2*cos(alv+th1+th2)]*G;
    qd = [xvd 
          yvd
          alvd
          th1d
          th2d];

    F = F + [0;0;0;T_f(1);T_f(2)];

    B = F - C*qd - Gmat;

    qdd = linsolve(M,B);

    xdot = zeros(10,1);
    xdot(1) = xvd;
    xdot(2) = qdd(1);
    xdot(3) = yvd;
    xdot(4) = qdd(2);
    xdot(5) = alvd;
    xdot(6) = qdd(3);
    xdot(7) = th1d;
    xdot(8) = qdd(4);
    xdot(9) = th2d;
    xdot(10) = qdd(5);
end