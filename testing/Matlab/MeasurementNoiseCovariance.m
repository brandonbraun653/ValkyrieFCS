clear all;
addpath('C:/git/GitHub/ValkyrieFCS/testing/Matlab/DataDumps/MeasurementNoiseCov');

R1=0; R2=8000;
C1=4; C2=9;


throttle{1} = csvread('ahrsLog_Throttle1210.csv',R1,C1, [R1,C1,R2,C2]);
throttle{2} = csvread('ahrsLog_Throttle1510.csv',R1,C1, [R1,C1,R2,C2]);

covLow = cov(throttle{1});
covHigh = cov(throttle{2});