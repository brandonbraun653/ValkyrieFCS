function [Pxz] = CrossCovariance(mean1, sigma1, mean2, sigma2, wc)
[N, ~] = size(sigma1);    
Pxz = zeros(N,N);

for i = 1:(2*N+1)
    Pxz = Pxz + wc(1,i)*(sigma1(:,i) - mean1)*(sigma2(:,i) - mean2)';
end