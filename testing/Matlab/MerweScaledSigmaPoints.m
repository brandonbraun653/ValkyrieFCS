function [Wc, Wm, Xsig] = MerweScaledSigmaPoints(N, alpha, beta, kappa, X, P)
%{
Inputs:
N - dimensionality of the problem

alpha - Controls the spread of the sigma points from the mean.
        A larger value equals a larger spread. A good range is 
        from 0 <= alpha <= 1.

beta - unknown what this controls. Good value is 2 for gaussian problems

kappa - unknown what this controls. Good value is 3-n

X - Current state estimate (Nx1)
P - Current covariance estimate (NxN)

Outpus:
Wc - Covariance Weights (1x(N+1))
Wm - Mean Weights (1x(N+1))
Xsig - Generated Sigma Points ((N+1)x(N))
%}
% Pg. 337 KBFP
lambda = alpha^2*(N+kappa)-N; 
maxPts = 2*N+1;

Wm = zeros(1,maxPts);
Wc = zeros(1,maxPts);
Xsig = zeros(N,maxPts);

%---------------------
% Sigma Points Generation
%---------------------
% Find the square root of the scaled P matrix using Cholesky decomposition
Psqrt = chol(P*(N+lambda));

% Fill in the sigma points
for i = 1:maxPts
    if i == 1
       Xsig(:,1) = X; 
    end

    if (2<=i && i<=(N+1))
       Xsig(:,i) = (X + Psqrt(:,i-1));
    end

    if ((N+1)<i)
       Xsig(:,i) = (X - Psqrt(:,i-N-1));
    end

end

%---------------------
% Weight Generation
%---------------------
% First weights
Wm(1) = lambda / (N + lambda);
Wc(1) = Wm(1) + 1 - alpha^2 + beta;

% Remaining weights
for i = 2:(maxPts)
    Wm(i) = 1/(2*(N+lambda));
    Wc(i) = Wm(i);
end % End for loop
end % End function