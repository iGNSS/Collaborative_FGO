clear;
close all;

%%%%%%%%%%trajectory
trajectory = 'trajectory.csv';
traj_data = csvread(trajectory);
traj_epoch = traj_data(:,1);
east_ref=traj_data(:,2);
north_ref = traj_data(:,3);
east_gnss=traj_data(:,4);
north_gnss = traj_data(:,5);

figure(1)
axis equal
plot(east_ref,north_ref,'k-','LineWidth',1.9 )
hold on;
plot(east_gnss,north_gnss,'ro','LineWidth',1.9 )
hold on;
grid on;
hold on;
axis equal
ax = gca;
ax.FontSize = 16; 
xlabel('east (meters)');
ylabel('north (meters)');
legend('\fontsize{16} GT','\fontsize{16} gnss');
title('\fontsize{16} trajectory')


%%%%%%%%%%%% error
error = 'error.csv';
error_data = csvread(error);
figure(2)
axis equal
error_epoch = error_data(:,1);
error_gnss=error_data(:,2);
error_gnss_3d=error_data(:,3);

plot(error_epoch,error_gnss,'r-o','LineWidth',1.9 )
hold on;
plot(error_epoch,error_gnss_3d,'g-o','LineWidth',1.9 )
hold on;
grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('error (meters)');
legend('\fontsize{16} gnss','\fontsize{16} error gnss 3d');
title('\fontsize{16} error')
mean(error_gnss)
mean(error_gnss_3d)

