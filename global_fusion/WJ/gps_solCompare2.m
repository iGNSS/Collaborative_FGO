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




traj_data2 = csvread('trajectory.csv');
traj_epoch2 = traj_data2(:,1);
traj_epoch2 = traj_epoch2 - traj_data2(1,1);

east_ref2=traj_data2(:,2);
north_ref2 = traj_data2(:,3);
east_gnss2=traj_data2(:,4);
north_gnss2 = traj_data2(:,5);


figure(1)
axis equal
plot(east_ref1,north_ref1,'k-','LineWidth',1.9 )
hold on;
plot(east_gnss1,north_gnss1,'ro','LineWidth',1.9 )
hold on;
plot(east_gnss2,north_gnss2,'bo-','LineWidth',1.9 )
hold on;
grid on;
hold on;
axis equal
ax = gca;
ax.FontSize = 16; 
xlabel('east (meters)');
ylabel('north (meters)');
legend('\fontsize{16} GT','\fontsize{16} WLS','\fontsize{16} FGO','\fontsize{16} FGO');
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

error_data2 = csvread('error.csv');
error_epoch2 = error_data2(:,1);
error_epoch2 =error_epoch2 -  error_data2(1,1);

error_gnss2=error_data2(:,2);
error_gnss2_3d=error_data2(:,3);


plot(error_epoch1,error_gnss1,'r-o','LineWidth',1.9 )
hold on;
plot(error_epoch2,error_gnss2,'b-o','LineWidth',1.9 )
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
legend('\fontsize{16} WLS','\fontsize{16} FGO','\fontsize{16} FGO-GNC');
title('\fontsize{16} error')
mean(error_gnss1)
mean(error_gnss2)
mean(error_gnss3)
sqrt(mean((error_gnss1).^2))
sqrt(mean((error_gnss2).^2))




