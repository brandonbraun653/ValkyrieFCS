function [X, P] = UnscentedTransform(sigmas, Wm, Wc, Usig)
%{
Inputs: 
sigma - Sigma points [Nx(2*N+1)]
Wm - Mean weights associated with sigma [1x(2*N+1)]
Wc - Covariance weights associated with sigma [1x(2*N+1)]
U - Noise/Uncertainty matrix [NxN]

Outputs:
X - New mean (Nx1)
P - New covariance (NxN)
%}
[N, ~] = size(sigmas);

% Calculate the new mean
X = zeros(N,1);
for i = 1:(2*N+1)
   X = X + Wm(1,i)*sigmas(:,i);
end

% Calculate the new covariance
P = zeros(N,N);
for i = 1:(2*N+1)
   P = P + Wc(1,i)*(sigmas(:,i)-X)*(sigmas(:,i)-X)';
end

% Add in the noise
P = P+Usig;
end