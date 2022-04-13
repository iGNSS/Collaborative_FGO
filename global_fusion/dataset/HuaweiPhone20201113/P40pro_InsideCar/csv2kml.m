clc;
clear all;
close all;

ublox_data = csvread('gnss_log_2020_11_13_11_11_59_nmea.csv');
% 
kmlwrite('gnss_log_2020_11_13_11_11_59_nmea.kml',ublox_data(:,2),ublox_data(:,3),'Icon',...
    'http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png','IconScale',0.5,'Color',[1,0,0],'Name','  ');
% csvtokml('conventional\data_con.kml',ublox_data,'cyan_dot');
