clc
clear all
close all
D2R = pi/180;R2D = 180/pi;
m2lat = 1/110734;
m2lon = 1/103043;
main_root='e:\';

mkdir('left_mi');mkdir('right_hw');mkdir('right_pixel');
mkdir('bag_hw');mkdir('bag_samsung');mkdir('emp');mkdir('left_hw');


folder=dir('*');

for idx=1:length(folder) 
    if  folder(idx).isdir==1
    rinex_file=[folder(idx).folder,'\',folder(idx).name];
   
    mkdir([[folder(idx).folder,'\',folder(idx).name,'\'],'0geo++\best'])
    
    mkdir([[folder(idx).folder,'\',folder(idx).name,'\'],'0geo++\more'])
    end
end

filename_obs=....
'E:\Dropbox\private\124_akf\210912_wj_gdd\right_pixel\20210912_182359\20210912_182359_rinex.txt';
filename_nav='E:\Dropbox\private\124_akf\210912_wj_gdd\emp\hksc2550.21n';
[data_wls_all,date_R,skymask]=....
    wls_renewed_2108_clear_pr(filename_nav,filename_obs,[],[],[]);%