function x1 = rdiff_kalman(x, t, sigma, strategy)
% RDIFF_KALMAN   Regularised numerical differentiation using method based
%                on Kalman filter.
%    X1 = RDIFF_KALMAN(X, T) returns an estimate of the first derivative
%    of X; if T is a scalar, it is used as spacing between abscissa; if T
%    is a vector, its elements are used as the abscissa. If X is a matrix,
%    RDIFF_KALMAN differentiates each of its columns independently.
%    
%    X1 = RDIFF_KALMAN(X, T, SIGMA), where SIGMA is a scalar, uses SIGMA
%    as an estimate of the variance of the errors corrupting the elements
%    of X under the assumption that those errors are independent and
%    identically distributed. If SIGMA is an NxN matrix, it is used as the
%    covariance matrix of those errors.
%    
%    X1 = RDIFF_KALMAN(X, T, SIGMA, STRATEGY) or
%    X1 = RDIFF_KALMAN(X, T, [], STRATEGY) uses the strategy for the
%    optimisation of the estimate of the regularisation parameter
%    (i.e. the parameter representative of the variance of the random
%    variables modelling the variations in the derivative) selected among
%    the following ones:
%       'dp'      based on discrepancy principle (requires SIGMA)
%       'ncp'     based on normalised cumulative periodogram
%    By default, the 'dp' strategy is used if SIGMA is provided, and the
%    'ncp' strategy otherwise.
%    
%    Jakub Wagner, February 2, 2020
%    Institute of Radioelectronics and Multimedia Technology
%    Warsaw University of Technology
% Check size of x
if isrow(x)
    x = x';
    xrow = true;
end
if isscalar(x)
    error('x must be a vector or a 2-D matrix.')
end
[N, R] = size(x);
% Check size of t
if isscalar(t), t = t*(0:N-1)'; end
if ~isvector(t) || length(t) ~= N
    error('for size(x) = [%d, %d], t should be a scalar or a vector of length %d; size(t) = [%d, %d]', size(x,1), size(x,2), N, size(t,1), size(t,2))
end
if isrow(t); t = t'; end
% Check availability and size of sigma
sigmaNotSpecified = false;
if ~exist('sigma', 'var') || isempty(sigma)
    if strcmpi(strategy, 'dp')
        warning('Cannot use ''%s'' strategy when sigma is not specified; using ''ncp'' strategy instead.', strategy)
        strategy = 'ncp';
    end
    sigma = 1;
    sigmaNotSpecified = true;
else
    if numel(sigma) == 1
        sigma = sigma*eye(N);
    elseif any(size(sigma) ~= N)
        error('Wrong size of sigma: [%d, %d] (should be scalar or %dx%d matrix)', size(sigma, 1), size(sigma, 2), N, N)
    end
end
% Select default strategy
if ~exist('strategy', 'var') || isempty(strategy)
    if sigmaNotSpecified
        strategy = 'ncp';
    else
        strategy = 'dp';
    end
end
% Considered values of the regularisation parameter
ss = sqrt(median(diag(sigma)))*logspace(-3, 3, 20)';
NS = numel(ss);
% Constants
switch lower(strategy)
    case 'dp'
        uDP = 2;
        trS = trace(sigma);
    case 'ncp'
        Nl = floor(N/2);
        l = (1:Nl)' / Nl;
    otherwise
        error('Unknown strategy: %s', strategy)
end
dtm = median(diff(t));
H = [1 0];
% Estimates of the derivative
x1 = zeros(N,R);
for r = 1:R     % for each sequence of data to be differentiated
    
    Xa = zeros(N, NS);   % results of approximation of data
    X1 = zeros(N, NS);   % estimates of derivative
    crit = zeros(NS, 1); % criterion for optimisation of reg. par.
    
    for ns = 1:NS   % for each considered value of reg. par.
        sa = ss(ns);
        
        % Results of filtering (smoothed data and derivative)
        xk = zeros(N,2);
        xk(1,1) = x(1,r);
        
        % Kalman filtering
        P = sa^2 * [dtm^4/4, dtm^3/2; dtm^3/2, dtm^2];
        for n = 2:N
            
            z = x(n,r);
            
            dt = t(n) - t(n-1);
            Fn = [1 dt; 0 1];
            
            xpre = Fn*xk(n-1,:)';
            Q = sa^2 * [dt^4/4, dt^3/2; dt^3/2, dt^2];
            Ppre = Fn*P*Fn' + Q;
            zest = H*xpre;
            if numel(sigma) == 1
                G = Ppre*H'/(H*Ppre*H' + sigma);
            else
                G = Ppre*H'/(H*Ppre*H' + sigma(n,n));
            end
            
            xk(n,:) = (xpre + G*(z - zest))';
            P = (eye(2) - G*H) * Ppre;
        end
        Xa(:,ns) = xk(:,1);
        X1(:,ns) = xk(:,2);
        
        % Compute criteria for optimisation of regularisation parameter
        switch lower(strategy)
            case 'dp'
                crit(ns) = abs(sum((Xa(:,ns) - x(:,r)).^2 - uDP*trS/N));
            case 'ncp'
                rf = fft(Xa(:,ns) - x(:,r));
                rp = abs(rf(1:Nl)).^2;
                rncp = cumsum(rp) / sum(rp);
                crit(ns) = sqrt(sum((rncp - l).^2));
        end
    end
    
    % Select optimum value of regularisation parameter
    nsSel = find(crit == min(crit), 1, 'first');
    x1(:,r) = [0; X1(2:end, nsSel)];
end
% Return row vector if x was one
if exist('xrow', 'var') && xrow
    x1 = x1';
end