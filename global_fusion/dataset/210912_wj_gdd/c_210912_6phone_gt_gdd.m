clc
clear all
close all
m2lat = 1/110734;
m2lon = 1/103043;
% file_root=dir('E:\Dropbox\private\124_akf\210930_qw\');
nmea_file=['E:\Dropbox\private\124_akf\210912_wj_gdd\right_hw\20210912_183157\20210912_183157_raw.txt'];
% nmea_pos=nmea_convert_20(nmea_file);%

nmea_pos=csvread(['E:\Dropbox\private\124_akf\210912_wj_gdd\right_hw\20210912_183157\20210912_183157_raw_nmea.csv']);
plot_osm_2020_start

geoplot(nmea_pos(:,2),nmea_pos(:,3),'r.','MarkerSize',25)
% 20210912_183157_rinex.txt
% filename_obs='E:\Dropbox\private\124_akf\210912_wj_gdd\right_hw\20210912_183157\20210912_183157_rinex.txt';
filename_obs='E:\Dropbox\private\124_akf\210912_wj_gdd\right_hw\0geo++\best\2021-09-12,18_31_54_GMT+08_00\0030255_all.21o';
filename_nav='E:\Dropbox\private\124_akf\210912_wj_gdd\emp\hksc2550.21n';

% new_rinex_file2=rinex_clear_IPNL(filename_obs);%
% data_wls_all_nlab=wls_renewed_2101(filename_nav,filename_obs);
% data_wls_all_nlab=data_wls_all_nlab(data_wls_all_nlab(:,2)>0,:);
% plot_osm_2020_start
% geoplot(data_wls_all_nlab(:,17),data_wls_all_nlab(:,18),'b.')
start_p=[ 22.313801,114.169864,4];
end_p=[ 22.317344,114.169140,4];
%  22.369388,114.114225,4
%   22.368460,114.115927,4
geolimits( 20*[-m2lat,m2lat]+sort([start_p(1),end_p(1)]),20*[-m2lat,m2lat]+sort([start_p(2),end_p(2)]))
  
% first roundee
start_time_bias=20210912190000;
time_step=[
    [00,00];[03,00];.....%
    [03,00];[09,11];.....%
    [09,11];[15,00];.....%
    [15,00];[21,13];.....%
    [21,13];[27,00];.....%
    [27,00];[33,09];.....%
    [33,09];[35,30];.....%
    [35,30];[41,44];.....%
    [41,44];[45,00];.....%
    [45,00];[51,22];.....%
    [51,22];[54,00];.....%
    [54,00];[100,22];.....%
    [100,22];[104,30];.....[100,22];[104,30];.....%[105,30];[105,50];
   ];.....%
    
%        [105,50];[108,00];.....%
%        [108,00];[109,30];.....%
% nmea_file=['E:\Dropbox\private\124_akf\210722_6phone\left_mi\20210722_201531\20210722_201531_raw.txt'];
%
% % nmea_pos=nmea_convert_20(nmea_file);%
% nmea_pos=csvread(['E:\Dropbox\private\124_akf\210722_6phone\left_mi\20210722_201531\20210722_201531_raw_nmea.csv']);
% nmea_pos(:,6)=nmea_pos(:,6)+80000;
% nmea_pos(:,1)=mod(nmea_pos(:,1),7*24*3600);

nmea_pos(:,6)=nmea_pos(:,6)+80000;
nmea_pos(:,1)=mod(nmea_pos(:,1),7*24*3600);

idx_count=zeros(1,4);
root=findstr(nmea_file,'\');

for idx=1:2:size(time_step,1)-1
    
    time_slip_s=start_time_bias+sum([time_step(idx,1),time_step(idx,2)].*[100,1]);
    
    time_slip_end=start_time_bias+sum([time_step(idx+1,1),time_step(idx+1,2)].*[100,1]);
    
    avail_index=logical( (nmea_pos(:,6)>=time_slip_s).* (nmea_pos(:,6)<=time_slip_end));
    %     if
    %
    %     end
    % geoplot(nmea_pos(avail_index,2),nmea_pos(avail_index,3),'r.')
    
    if sum( idx==[1:8:41])
        nmea_pos(avail_index,7)=1;
        
        nmea_pos(avail_index, 8:10)=repmat( start_p,sum(avail_index) ,1);
        idx_count(1,1)=idx_count(1,1)+1;
        text=['Starting P No.' num2str(idx_count(1,1))];
    end
    if sum( idx==[1:8:41]+2)
        nmea_pos(avail_index,7)=2;
        nmea_pos(avail_index, 8:10)=....
            linspace_whole(start_p,end_p,sum(avail_index)) ;
        idx_count(1,2)=idx_count(1,2)+1;
        text=['Down Path No.' num2str(idx_count(1,2))];
    end
    if sum( idx==[1:8:41]+4)
        nmea_pos(avail_index,7)=3;
        nmea_pos(avail_index, 8:10)=repmat( end_p,sum(avail_index) ,1);
        idx_count(1,3)=idx_count(1,3)+1;
        text=['Ending P No.' num2str(idx_count(1,3))];
    end
    if sum( idx==[1:8:41]+6)
        nmea_pos(avail_index,7)=4;
        nmea_pos(avail_index, 8:10)=....
            linspace_whole(end_p,start_p,sum(avail_index)) ;
        idx_count(1,4)=idx_count(1,4)+1;
        text=['Up Path No.' num2str(idx_count(1,4))];
        
    end
    
    text=[text,'-sT-',[num2str(time_slip_s)]];
    plot_osm_2020_start
    geoplot(nmea_pos(avail_index,2),nmea_pos(avail_index,3),'r.','MarkerSize',25)
    geoplot(nmea_pos(avail_index,8),nmea_pos(avail_index,9),'B.','MarkerSize',25)
    title(text)
    geolimits( 20*[-m2lat,m2lat]+sort([start_p(1),end_p(1)]),20*[-m2lat,m2lat]+sort([start_p(2),end_p(2)]))
    
    saveas(gcf,[nmea_file(1:root(end)),'4' num2str(idx,'%05.2f') text '.jpg'] )
    close all
end
clc
gif_maker([nmea_file(1:root(end)),'4*.jpg'] )


dlmwrite([nmea_file(1:root(end)) 'GT1.csv'],nmea_pos, 'precision','%10.15f')
clc




%%%%%%%%%%%%%%%%%%%PAth2

start_p=[ 22.299302,114.168476,5];
end_p=[ 22.296094,114.169180,5];
%  22.369388,114.114225,4
%   22.368460,114.115927,4

% first round
start_time_bias=20210912210000;
time_step=[
    [-5,00];[2,30]
    [2,30];[8,05];.....%
    [8,05];[10,30];.....%
    [10,30];[15,48];.....%
    [15,48];[18,30];.....%
    [18,30];[24,00];.....%
    [24,00];[28,43];.....%
    ];.....% [27,00];[];.....%
   
%     [28,43];[30,30];.....%
%     [30,30];[33,30];.....%
%       [37,23];[40,30];.....%
%     [41,00];[43,00];.....%
%     [];.....%[105,30];[105,50];
% nmea_file=['E:\Dropbox\private\124_akf\210722_6phone\left_mi\20210722_201531\20210722_201531_raw.txt'];
%
% % nmea_pos=nmea_convert_20(nmea_file);%
% nmea_pos=csvread(['E:\Dropbox\private\124_akf\210722_6phone\left_mi\20210722_201531\20210722_201531_raw_nmea.csv']);
% nmea_pos(:,6)=nmea_pos(:,6)+80000;
% nmea_pos(:,1)=mod(nmea_pos(:,1),7*24*3600);

% nmea_pos(:,6)=nmea_pos(:,6)+80000;
% nmea_pos(:,1)=mod(nmea_pos(:,1),7*24*3600);

idx_count=zeros(1,4);
root=findstr(nmea_file,'\');

for idx=1:2:size(time_step,1)-1
    
    time_slip_s=start_time_bias+sum([time_step(idx,1),time_step(idx,2)].*[100,1]);
    
    time_slip_end=start_time_bias+sum([time_step(idx+1,1),time_step(idx+1,2)].*[100,1]);
    
    avail_index=logical( (nmea_pos(:,6)>=time_slip_s).* (nmea_pos(:,6)<=time_slip_end));
    %     if
    %
    %     end
    % geoplot(nmea_pos(avail_index,2),nmea_pos(avail_index,3),'r.')
    
    if sum( idx==[1:8:41])
        nmea_pos(avail_index,7)=1;
        
        nmea_pos(avail_index, 8:10)=repmat( start_p,sum(avail_index) ,1);
        idx_count(1,1)=idx_count(1,1)+1;
        text=['Starting P No.' num2str(idx_count(1,1))];
    end
    if sum( idx==[1:8:41]+2)
        nmea_pos(avail_index,7)=2;
        nmea_pos(avail_index, 8:10)=....
            linspace_whole(start_p,end_p,sum(avail_index)) ;
        idx_count(1,2)=idx_count(1,2)+1;
        text=['Down Path No.' num2str(idx_count(1,2))];
    end
    if sum( idx==[1:8:41]+4)
        nmea_pos(avail_index,7)=3;
        nmea_pos(avail_index, 8:10)=repmat( end_p,sum(avail_index) ,1);
        idx_count(1,3)=idx_count(1,3)+1;
        text=['Ending P No.' num2str(idx_count(1,3))];
    end
    if sum( idx==[1:8:41]+6)
        nmea_pos(avail_index,7)=4;
        nmea_pos(avail_index, 8:10)=....
            linspace_whole(end_p,start_p,sum(avail_index)) ;
        idx_count(1,4)=idx_count(1,4)+1;
        text=['Up Path No.' num2str(idx_count(1,4))];
        
    end
    text=[text,'-sT-',[num2str(time_slip_s)]];
    plot_osm_2020_start
    geoplot(nmea_pos(avail_index,2),nmea_pos(avail_index,3),'r.','MarkerSize',25)
    geoplot(nmea_pos(avail_index,8),nmea_pos(avail_index,9),'B.','MarkerSize',25)
    title(text)
    pause(1)
    geolimits( 20*[-m2lat,m2lat]+sort([start_p(1),end_p(1)]),20*[-m2lat,m2lat]+sort([start_p(2),end_p(2)]))
    saveas(gcf,[nmea_file(1:root(end)),'5' num2str(idx,'%05.2f') text '.jpg'] )
    close all
end
clc
gif_maker([nmea_file(1:root(end)),'5*.jpg'] )


dlmwrite([nmea_file(1:root(end)) 'GT2.csv'],nmea_pos, 'precision','%10.15f')
clc