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

traj_data3 = csvread('EKFtrajectory.csv');
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
plot(east_gnss3,north_gnss3,'go','LineWidth',1.9 )
hold on;
plot(east_gnss2,north_gnss2,'bo','LineWidth',1.9 )
hold on;
grid on;
hold on;
axis equal
ax = gca;
ax.FontSize = 16; 
xlabel('east (meters)');
ylabel('north (meters)');
legend('\fontsize{16} GT','\fontsize{16} WLS','\fontsize{16} EKF','\fontsize{16} FGO');
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

error_data3 = csvread('EKFerror.csv');
error_epoch3 = error_data3(:,1);
error_epoch3 =error_epoch3 -  error_data3(1,1);

error_gnss3 = error_data3(:,2);
error_gnss3_3d = error_data3(:,3);


plot(error_epoch1,error_gnss1,'r-o','LineWidth',1.9 )
hold on;
plot(error_epoch2,error_gnss2,'b-o','LineWidth',1.9 )
hold on;
plot(error_epoch3,error_gnss3,'g-o','LineWidth',1.9 )
hold on;
% plot(errorEX_epoch,errorEX_gnss_3d,'g-o','LineWidth',1.9 )
% hold on;

grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlabel('time (seconds)');
ylabel('error (meters)');
legend('\fontsize{16} WLS','\fontsize{16} FGO','\fontsize{16} EKF');
title('\fontsize{16} error')
mean(error_gnss1)
mean(error_gnss2)
sqrt(mean((error_gnss1).^2));
% mean(error_gnss_3d)
subplot(2,1,2) % plot number of satellites
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



figure(3)
subplot(5,1,1) % plot error
plot(error_epoch1,error_gnss1,'r-o','LineWidth',1.9 )
hold on;
plot(error_epoch2,error_gnss2,'b-o','LineWidth',1.9 )
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




%calculate confidence region
% cepup = meanerror1 + stderror1;
% cepdown = meanerror1 - stderror1;
% cepfgoup = meanerror2 + stderror2;
% cepfgodown = meanerror2 - stderror2;

%plot confidence region
error_east_gnss1 = east_ref1 - east_gnss1;
error_north_gnss1 = north_ref1 - north_gnss1;
error_east_gnss2 = east_ref2 - east_gnss2;
error_north_gnss2 = north_ref2 - north_gnss2;
error_east_gnss3 = east_ref3 - east_gnss3;
error_north_gnss3 = north_ref3 - north_gnss3;
% figure(4)
% axis equal
% plot(error_east_gnss1,error_north_gnss1,'ro','LineWidth',1.9 )
% hold on;
% plot(error_east_gnss3,error_north_gnss3,'go','LineWidth',1.9 )
% hold on;
% plot(error_east_gnss2,error_north_gnss2,'bo','LineWidth',1.9 )
% hold on;
% grid on;
% hold on;
% axis equal
% ax = gca;
% ax.FontSize = 16; 
% xlabel('east (meters)');
% ylabel('north (meters)');
% legend('\fontsize{16} WLS','\fontsize{16} EKF','\fontsize{16} FGO');
% title('\fontsize{16} trajectory')

global result_fgo_ab;
global result_wls_ab;
global result_ekf_ab;

figure(5)
axis equal;
hold on;
grid on;
hold on;
ax = gca;
ax.FontSize = 16; 
xlabel('error-east (meters)');
ylabel('error-north (meters)');
title('\fontsize{16} CEP95')
A = [error_east_gnss1(~isnan(error_east_gnss1)),error_north_gnss1(~isnan(error_north_gnss1))];
B = [error_east_gnss2(~isnan(error_east_gnss2)),error_north_gnss2(~isnan(error_north_gnss2))];
C = [error_east_gnss3(~isnan(error_east_gnss3)),error_north_gnss3(~isnan(error_north_gnss3))];
p = 0.95;
errorEllipse(A,p);
hold on;
errorEllipse2(B,p);
errorEllipse3(C,p);
hold on;
legend('\fontsize{16} WLS-CEP','\fontsize{16} WLS','\fontsize{16} FGO-CEP','\fontsize{16} FGO','\fontsize{16} EKF-CEP','\fontsize{16} EKF');
hold on;
axis equal;


%error analysis
meanerror1 = nanmean(error_gnss1);
meanerror2 = mean(error_gnss2);
meanerror3 = mean(error_gnss3);
rmserror1 = rms(error_gnss1,'omitnan');
rmserror2 = rms(error_gnss2);
rmserror3 = rms(error_gnss3);
stderror1 = std(error_gnss1,'omitnan');
stderror2 = std(error_gnss2);
stderror3 = std(error_gnss3);
maxerror1 = max(error_gnss1);
maxerror2 = max(error_gnss2);
maxerror3 = max(error_gnss3);
minerror1 = min(error_gnss1);
minerror2 = min(error_gnss2);
minerror3 = min(error_gnss3);
M = [meanerror1,meanerror3,meanerror2,(meanerror1-meanerror3)/meanerror1,abs(meanerror1-meanerror2)/meanerror1];
R = [rmserror1,rmserror3,rmserror2,(rmserror1-rmserror3)/rmserror1,abs(rmserror1-rmserror2)/rmserror1];
S = [stderror1,stderror3,stderror2,(stderror1-stderror3)/stderror1,abs(stderror1-stderror2)/stderror1];
X = [maxerror1,maxerror3,maxerror2,(maxerror1-maxerror3)/maxerror1,abs(maxerror1-maxerror2)/maxerror1];
N = [minerror1,minerror3,minerror2,(minerror1-minerror3)/minerror1,abs(minerror1-minerror2)/minerror1];
rEa_wls = result_wls_ab(:,1);
rEb_wls = result_wls_ab(:,2);
rEa_fgo = result_fgo_ab(:,1);
rEb_fgo = result_fgo_ab(:,2);
rEa_ekf = result_ekf_ab(:,1);
rEb_ekf = result_ekf_ab(:,2);
rEa_error=[rEa_wls,rEa_ekf,rEa_fgo,(rEa_wls-rEa_ekf)/rEa_wls,(rEa_wls-rEa_fgo)/rEa_wls];
rEb_error=[rEb_wls,rEb_ekf,rEb_fgo,(rEb_wls-rEb_ekf)/rEb_wls,(rEb_wls-rEb_fgo)/rEb_wls];
result = [R;M;S;X;N;rEa_error;rEb_error];
csvwrite('results.csv',transpose(result))




function errorEllipse(datamatrix,p)
%print 2-demension confidence ellipse
%In:n¡Á2 matrix, confidence probability p
global result_wls_ab;
data = datamatrix;
covariance = cov(data);
[eigenvec,eigenval] = eig(covariance);

[sortEigenval,index] = sort(diag(eigenval),'descend');
sortEigenvec = eigenvec(:,index);

largestEigenval = sortEigenval(1);
smallestEigenval = sortEigenval(end); %find the minimum eigenvalue
largestEigenvec = sortEigenvec(:,1); %find the maximum eigenvector

angle = atan2(largestEigenvec(2), largestEigenvec(1)); %calculate the angle between x-axis and the maximum eigenvector, [-pi,pi]

if(angle < 0) 
    angle = angle + 2*pi;
end

avg = mean(data); %calculate the mean of two columns of data

%configure the parameters of the confidence ellipse
chisquareVal = sqrt(chi2inv(p,2)); %chi-square value
thetaGrid = linspace(0,2*pi); 
phi = angle; %rotation angle
X0=avg(1);
Y0=avg(2); 
a=chisquareVal*sqrt(largestEigenval); %the wheelbase length
b=chisquareVal*sqrt(smallestEigenval);
result_wls_ab = [a,b]
csvwrite('wls_ab.csv',transpose(result_wls_ab));
ellipseXR = a*cos( thetaGrid ); %onto rectangular axis
ellipseYR = b*sin( thetaGrid );


R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ]; %rotation matrix

rEllipse = [ellipseXR;ellipseYR]' * R; %rotation

plot(rEllipse(:,1) + X0,rEllipse(:,2) + Y0,'-','LineWidth',1.9) %print
plot(data(:,1), data(:,2), '.');

axis square
end

function errorEllipse2(datamatrix,p)
%print 2-demension confidence ellipse
%In:n¡Á2 matrix, confidence probability p
global result_fgo_ab;
data = datamatrix;
covariance = cov(data);
[eigenvec,eigenval] = eig(covariance);

[sortEigenval,index] = sort(diag(eigenval),'descend');
sortEigenvec = eigenvec(:,index);

largestEigenval = sortEigenval(1);
smallestEigenval = sortEigenval(end); %find the minimum eigenvalue
largestEigenvec = sortEigenvec(:,1); %find the maximum eigenvector

angle = atan2(largestEigenvec(2), largestEigenvec(1)); %calculate the angle between x-axis and the maximum eigenvector, [-pi,pi]

if(angle < 0) 
    angle = angle + 2*pi;
end

avg = mean(data); %calculate the mean of two columns of data

%configure the parameters of the confidence ellipse
chisquareVal = sqrt(chi2inv(p,2)); %chi-square value
thetaGrid = linspace(0,2*pi); 
phi = angle; %rotation angle
X0=avg(1);
Y0=avg(2); 
a=chisquareVal*sqrt(largestEigenval); %the wheelbase length
b=chisquareVal*sqrt(smallestEigenval);
result_fgo_ab = [a,b];
csvwrite('fgo_ab.csv',transpose(result_fgo_ab));
ellipseXR = a*cos( thetaGrid ); %onto rectangular axis
ellipseYR = b*sin( thetaGrid );

R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ]; %rotation matrix

rEllipse = [ellipseXR;ellipseYR]' * R; %rotation

plot(rEllipse(:,1) + X0,rEllipse(:,2) + Y0,'r-','LineWidth',1.9)
plot(data(:,1), data(:,2), '.')%print




axis square
end
function errorEllipse3(datamatrix,p)
%print 2-demension confidence ellipse
%In:n¡Á2 matrix, confidence probability p
global result_ekf_ab;
data = datamatrix;
covariance = cov(data);
[eigenvec,eigenval] = eig(covariance);

[sortEigenval,index] = sort(diag(eigenval),'descend');
sortEigenvec = eigenvec(:,index);

largestEigenval = sortEigenval(1);
smallestEigenval = sortEigenval(end); %find the minimum eigenvalue
largestEigenvec = sortEigenvec(:,1); %find the maximum eigenvector

angle = atan2(largestEigenvec(2), largestEigenvec(1)); %calculate the angle between x-axis and the maximum eigenvector, [-pi,pi]

if(angle < 0) 
    angle = angle + 2*pi;
end

avg = mean(data); %calculate the mean of two columns of data

%configure the parameters of the confidence ellipse
chisquareVal = sqrt(chi2inv(p,2)); %chi-square value
thetaGrid = linspace(0,2*pi); 
phi = angle; %rotation angle
X0=avg(1);
Y0=avg(2); 
a=chisquareVal*sqrt(largestEigenval); %the wheelbase length
b=chisquareVal*sqrt(smallestEigenval);
result_ekf_ab = [a,b];
csvwrite('ekf_ab.csv',transpose(result_ekf_ab));
ellipseXR = a*cos( thetaGrid ); %onto rectangular axis
ellipseYR = b*sin( thetaGrid );

R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ]; %rotation matrix

rEllipse = [ellipseXR;ellipseYR]' * R; %rotation

plot(rEllipse(:,1) + X0,rEllipse(:,2) + Y0,'g-','LineWidth',1.9)
plot(data(:,1), data(:,2), '.')%print




axis square
end
