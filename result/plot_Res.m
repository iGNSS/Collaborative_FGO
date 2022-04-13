clear;
close all;

%%%%%%%%%%Residual
resFile = 'pseudorange_residual2.csv';
resFile_data = csvread(resFile);
res_time = resFile_data(:,1);
res_pr=resFile_data(:,2);


figure(1)
% axis equal
subplot(2,1,1)
plot(res_time,res_pr,'k-','LineWidth',0.9 )
hold on;
% plot(res_time,res_v,'ro','LineWidth',0.9 )
% hold on;
grid on;
hold on;
% axis equal
ax = gca;
ax.FontSize = 16; 
xlabel('time');
ylabel('residual value');
legend('\fontsize{16} res_u','\fontsize{16} res_v');
title('\fontsize{16} residual')
mean(res_pr)


