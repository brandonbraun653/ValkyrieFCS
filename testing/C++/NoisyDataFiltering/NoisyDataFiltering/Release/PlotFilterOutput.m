
inputData = csvread('inputData.csv',0,0);
filteredData = csvread('filteredData.csv',0,0);

% Acceleration in X
%subplot(3,1,1); 
figure(1); clf(1);
plot(inputData(:,1),'g--'); hold on; 
plot(filteredData(:,1), 'b', 'LineWidth', 1);

title('UKF Acceleration Data X');
xlabel('Data Points');
ylabel('Acceleration (m/s^2)');
legend('Unfiltered','Filtered');
grid on;