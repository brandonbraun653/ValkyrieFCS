function [Xout] = UnscentedKalmanFilter(dMeas, X, Phi, B, U, P, Q, R, H, merwe_const)
%{
INPUTS
dMeas: Measured data (NxM)
    All rows must be filled with some sort of data, even if just zeros.
    The filter will be applied to the entire data set.
       
X: Model state vector, also called the mean. (Nx1)
Phi: Model state transition matrix (NxN)

U: Model Input vector (Nx1)
B: Model Input transition matrix (NxN)

P: State covariance/uncertainty matrix (NxN)
R: Measurement covariance/uncertainty matrix (NxN)
Q: Process noise matrix (NxN)
H: Measurement selection matrix (NxN)

%}

% Figure out dimensionalities
[N, ~] = size(X);
[~, steps] = size(dMeas);

% Rename the inputs for consistency
Xk = X;
Pk = P;
alpha = merwe_const(1);
beta = merwe_const(2);
kappa = merwe_const(3);

% Initialize variables
Xout = zeros(N,steps);

for k = 1:steps
    %---------------------
    % 1. Predict Step
    %---------------------
    % a) Generate sigma points and weights
    [Wc, Wm, Xsig] = MerweScaledSigmaPoints(N, alpha, beta, kappa, Xk, Pk);

    % b) Project selected sigma points through model
    Ysig = zeros(N, 2*N+1);
    for i = 1:(2*N+1)
       Ysig(:,i) = Phi*Xsig(:,i) + B*U(:,i);
    end

    % c) Compute the mean and covariance of the prior using UT
    [Xp, Pp] = UnscentedTransform(Ysig, Wm, Wc, Q);
    
    %---------------------
    % 2. Update Step
    %---------------------
    
    % a) Convert the sigma points from (1b) into a measurement
    Zsig = H*Ysig;
    
    % b) Compute the mean and covariance of the measurement 
    [Xz, Pz] = UnscentedTransform(Zsig, Wm, Wc, R);
    
    % c) Compute the residual between measured and predicted
    Yk = dMeas(:,k) - Xz;
    
    % d) Compute the cross covariance of the state and measurement
    Pxz = CrossCovariance(Xp, Ysig, Xz, Zsig, Wc);
    
    % e) Compute the Kalman Gain
    K = Pxz*pinv(Pz);
    
    % f) Compute the new state estimate and covariance
    Xk = Xp + K*Yk;
    Pk = Pp - K*Pz*K';
    
    %---------------------
    % 3. Logging
    %---------------------
    Xout(:,k) = Xk;
end % End for loop
end % End function