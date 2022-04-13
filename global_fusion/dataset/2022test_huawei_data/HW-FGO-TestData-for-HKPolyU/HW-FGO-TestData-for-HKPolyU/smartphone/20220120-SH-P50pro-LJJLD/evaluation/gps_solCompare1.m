clear;
close all;

%%%%%%%%%%trajectory
traj_data1 = csvread('trajectoryBase.csv');
traj_epoch1 = traj_data1(:,1);
traj_epoch1 = traj_epoch1 - traj_data1(1,1);

east_ref1=traj_data1(:,2);
north_ref1 = traj_data1(:,3);
east_gnss1=traj_data1(:,4);
north_gnss1 = traj_data1(:,5);




traj_data2 = csvread('trajectoryPVT.csv');
traj_epoch2 = traj_data2(:,1);
traj_epoch2 = traj_epoch2 - traj_data2(1,1);

east_ref2=traj_data2(:,2);
north_ref2 = traj_data2(:,3);
east_gnss2=traj_data2(:,4);
north_gnss2 = traj_data2(:,5);

traj_data3 = csvread('trajectory.csv');
traj_epoch3 = traj_data3(:,1);
traj_epoch3 = traj_epoch3 - traj_data3(1,1);

east_ref3=traj_data3(:,2);
north_ref3 = traj_data3(:,3);
east_gnss3=traj_data3(:,4);
north_gnss3 = traj_data3(:,5);


trajMoyion_data           = csvread('FGO_trajectoryllh_psr_dop_fusion.csv');
trajMoyion_data_epoch     = trajMoyion_data(:,1);
trajMoyion_data_epoch     = trajMoyion_data_epoch - trajMoyion_data(1,1);

vx                        =trajMoyion_data(:,6);
vy                        =trajMoyion_data(:,7);
vz                        =trajMoyion_data(:,8);
ax                        =trajMoyion_data(:,9);
ay                        =trajMoyion_data(:,10);
az                        =trajMoyion_data(:,11);
jx                        =trajMoyion_data(:,12);
jy                        =trajMoyion_data(:,13);
jz                        =trajMoyion_data(:,14);
sn                        =trajMoyion_data(:,15);

%EKF
% trajMoyion_dataE           = csvread('EKF_trajectoryllh_psr_dop_fusion.csv');
% trajMoyion_data_epochE     = trajMoyion_dataE(:,1);
% trajMoyion_data_epochE     = trajMoyion_data_epochE - trajMoyion_dataE(1,1);
% 
% vxE                        =trajMoyion_dataE(:,6);
% vyE                        =trajMoyion_dataE(:,7);
% vzE                        =trajMoyion_dataE(:,8);
% axE                        =trajMoyion_dataE(:,9);
% ayE                        =trajMoyion_dataE(:,10);
% azE                        =trajMoyion_dataE(:,11);
% jxE                        =trajMoyion_dataE(:,12);
% jyE                        =trajMoyion_dataE(:,13);
% jzE                        =trajMoyion_dataE(:,14);
% snE                        =trajMoyion_dataE(:,15);

figure(1)
axis equal
plot(east_ref1,north_ref1,'k-','LineWidth',1.9 )
hold on;
plot(east_gnss1,north_gnss1,'ro','LineWidth',1.9 )
hold on;
plot(east_gnss2,north_gnss2,'go-','LineWidth',1.9 )
hold on;
plot(east_gnss3,north_gnss3,'bo-','LineWidth',1.9 )
hold on;
grid on;
hold on;
axis equal
ax = gca;
ax.FontSize = 16; 
xlabel('east (meters)');
ylabel('north (meters)');
legend('\fontsize{16} GT','\fontsize{16} WLS','\fontsize{16} PVT','\fontsize{16} FGO');
title('\fontsize{16} trajectory')


%%%%%%%%%%%% error
error_data1 = csvread('errorBase.csv');
figure(2)
subplot(2,1,1) % plot number of satellites
axis equal
error_epoch1 = error_data1(:,1);
error_epoch1 = error_epoch1 - error_data1(1,1);
error_gnss1=error_data1(:,2);
error_gnss1_3d=error_data1(:,3);

error_data2 = csvread('errorPVT.csv');
error_epoch2 = error_data2(:,1);
error_epoch2 =error_epoch2 -  error_data2(1,1);

error_gnss2=error_data2(:,2);
error_gnss2_3d=error_data2(:,3);

error_data3 = csvread('error.csv');
error_epoch3 = error_data3(:,1);
error_epoch3 =error_epoch3 -  error_data3(1,1);

error_gnss3 = error_data3(:,2);
error_gnss3_3d = error_data3(:,3);


plot(error_epoch1,error_gnss1,'r-o','LineWidth',1.9 )
hold on;
plot(error_epoch2,error_gnss2,'g-o','LineWidth',1.9 )
hold on;
plot(error_epoch3,error_gnss3,'b-o','LineWidth',1.9 )
hold on;
% plot(errorEX_epoch,errorEX_gnss_3d,'g-o','LineWidth',1.9 )
% hold on;

grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
ylim([0 60])
xlabel('time (seconds)');
ylabel('error (meters)');
legend('\fontsize{16} WLS','\fontsize{16} PVT','\fontsize{16} FGO');
title('\fontsize{16} error')
mean(error_gnss1)
mean(error_gnss2)
mean(error_gnss3)
% sqrt(mean((error_gnss1).^2))
% sqrt(mean((error_gnss2).^2))
% sqrt(mean((error_gnss3).^2))

% mean(error_gnss_3d)
subplot(2,1,2) % plot number of satellites
plot(trajMoyion_data_epoch,sn,'r-','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlim([0 1500])

xlabel('time (seconds)');
ylabel('number of sat');
legend('\fontsize{16} number of sat');
title('\fontsize{16} number of sat')



figure(3)
subplot(5,1,1) % plot error
plot(error_epoch1,error_gnss1,'r-o','LineWidth',1.9 )
hold on;
plot(error_epoch3,error_gnss3,'b-o','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('error (meters)');
legend('\fontsize{16} WLS','\fontsize{16} FGO');
title('\fontsize{16} error')

subplot(5,1,2) % plot velocity
plot(trajMoyion_data_epoch,vx,'r-o','LineWidth',1.9 )
hold on;
plot(trajMoyion_data_epoch,vy,'g-o','LineWidth',1.9 )
hold on;
plot(trajMoyion_data_epoch,vz,'b-o','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;
ylim([-6 6])
ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('velocity (m/s)');
legend('\fontsize{16} x','\fontsize{16} y','\fontsize{16} z');
title('\fontsize{16} velocity')

subplot(5,1,3) % plot acceleration
plot(trajMoyion_data_epoch,ax,'r-o','LineWidth',1.9 )
hold on;
plot(trajMoyion_data_epoch,ay,'g-o','LineWidth',1.9 )
hold on;
plot(trajMoyion_data_epoch,az,'b-o','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('acceleration (m/s^2)');
legend('\fontsize{16} x','\fontsize{16} y','\fontsize{16} z');
title('\fontsize{16} acceleration')


subplot(5,1,4) % plot jerk
plot(trajMoyion_data_epoch,jx,'r-','LineWidth',1.9 )
hold on;
plot(trajMoyion_data_epoch,jy,'g-','LineWidth',1.9 )
hold on;
plot(trajMoyion_data_epoch,jz,'b-','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('jerk (m/s^3)');
legend('\fontsize{16} x','\fontsize{16} y','\fontsize{16} z');
title('\fontsize{16} jerk')

subplot(5,1,5) % plot number of satellites
plot(trajMoyion_data_epoch,sn,'r-','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;

ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('number of sat');
legend('\fontsize{16} number of sat');
title('\fontsize{16} number of sat')





