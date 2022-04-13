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
    'E:\Dropbox\private\124_akf\211106_campus_RUNING\left_hw\0geo++\more\BA01310H.21o';
filename_nav='E:\Dropbox\private\124_akf\211106_campus_RUNING\emp\hksc310h.21n';
[data_wls_all,date_R,skymask]=....
    wls_renewed_2108_clear_pr(filename_nav,filename_obs,[],[],[]);%


%
nmea_file=['E:\Dropbox\private\124_akf\211106_campus_RUNING\left_hw\20211106_152548\20211106_152548_raw.txt'];


% nmea_file=['E:\Dropbox\private\124_akf\211106_campus_RUNING\sst_hw\20211106_152504\20211106_152504_raw.txt'];

nmea_pos=nmea_convert_20(nmea_file);%
% nmea_pos=nmea_convert_20(nmea_file);%
plot_osm_2020_start
geoplot(nmea_pos(:,2),nmea_pos(:,3),'r.','MarkerSize',25)

% geoplot(data_wls_all_nlab(:,17),data_wls_all_nlab(:,18),'b.')
data_wls_all=data_wls_all(data_wls_all(:,2)>0,:);
geoplot(data_wls_all(:,19),data_wls_all(:,18),'b.')


start_p=[ 22.304184,114.178423,9.90];
end_p=[22.3049196,114.178610883333,9.90];

geolimits( 20*[-m2lat,m2lat]+sort([start_p(1),end_p(1)]),20*[-m2lat,m2lat]+sort([start_p(2),end_p(2)]))

start_time_bias=20211106150000;2021112190000;%loacl time
time_step=[
    [25,00];[30,30];.....%
    [30,30];[30,57];.....%
    [30,57];[30,57];.....%
    [30,42];[31,24];.....%
    [31,24];[31,24];.....%'
    [31,24];[31,53];.....%
    [31,53]; [31,53];.....%
    [31,53];[32,22];.....%
    [32,22];[32,22];.....%.....%
    [32,22];[32,53.5];.....%
    [32,53.5];[32,53.5];.....%.....%
    [32,53.5];[	33,25];.....%
    [33,25];[33,25];.....%.....%
    [33,25];[47,30];.....[100,22];[104,30];.....%[105,30];[105,50];
   ];.....%

 close all

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
  

