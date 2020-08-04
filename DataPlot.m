clear
cd road_test_RC_4444_Jul30_J7_7
%mkdir PlotOutput
Path='./';
BagFiles = dir(fullfile(Path,'*.bag'));
for filenum=2:length(BagFiles)
  filename = BagFiles(filenum).name(1:length(BagFiles(filenum).name)-4);
  csvname_fusion = [filename, 'Camera.csv'];
  csvname_lead = [filename, 'Lead.csv'];
  csvname_l_img = [filename, 'LImage.csv'];
  csvname_r_img = [filename, 'RImage.csv'];
  csvname_odom = [filename, 'Odom.csv'];
  csvname_radar = [filename, 'Radar.csv'];
  csvname_l_lane = [filename, 'LaneL.csv'];
  csvname_r_lane = [filename, 'LaneR.csv'];

  data_fusion  = importdata(csvname_fusion,',',1);
  data_lead  = importdata(csvname_lead,',',1);
  data_l_img  = importdata(csvname_l_img,',',1);
  data_r_img  = importdata(csvname_r_img,',',1);
  data_odom  = importdata(csvname_odom,',',1);
  data_radar  = importdata(csvname_radar,',',1);
  data_l_lane = importdata(csvname_l_lane,',',1);
  data_r_lane = importdata(csvname_r_lane,',',1);

  time_min = min([data_fusion.data(1,1), data_lead.data(1,1), data_l_img.data(1,1), data_r_img.data(1,1), data_odom.data(1,1), data_radar.data(1,1)])...
                 + 5000000; % add 5ms means in the beginning of rosbag, the intervals between every frame is uneven 
  time_max = max([data_fusion.data(length(data_fusion.data),1), data_lead.data(length(data_lead.data),1), data_l_img.data(length(data_l_img.data),1), ...
                 data_r_img.data(length(data_r_img.data),1), data_odom.data(length(data_odom.data),1), data_radar.data(length(data_radar.data),1)]);                
  t_interval = 50000000; % 50ms
  num_interval = floor((time_max - time_min)/t_interval);

  timeline_fusion = unique(data_fusion.data(:,1));
  timeline_lead = data_lead.data(:,1);
  timeline_l_img = data_l_img.data(:,1);
  timeline_r_img = data_r_img.data(:,1);
  timeline_odom = data_odom.data(:,1);
  timeline_radar = unique(data_radar.data(:,1));
  timeline_l_lane = unique(data_l_lane.data(:,1));
  timeline_r_lane = unique(data_r_lane.data(:,1));
  
  UFlag = 0;
  hasFu = 0;
  hasLe = 0;
  hasOd = 0;
  hasRa = 0;
  hasLLa = 0;
  hasRLa = 0;
  hasLI = 0;
  hasRI = 0;

  for i=0:num_interval    
    for j=1:length(timeline_fusion)
      if ((timeline_fusion(j) >= time_min+i*t_interval) && (timeline_fusion(j) < time_min+(i+1)*t_interval))
        UFlag = UFlag + 1;
        hasFu = 1;
        CurrentFusionPosX = data_fusion.data(data_fusion.data(:,1)==timeline_fusion(j),4);
        CurrentFusionPosY = data_fusion.data(data_fusion.data(:,1)==timeline_fusion(j),5);
        CurrentFusionID = data_fusion.data(data_fusion.data(:,1)==timeline_fusion(j),2);
        CurrentFusionNUM = length(CurrentFusionID);
      endif
    endfor
    
    for j=1:length(timeline_lead)
      if ((timeline_lead(j) >= time_min+i*t_interval) && (timeline_lead(j) < time_min+(i+1)*t_interval))      
        UFlag = UFlag + 1;
        hasLe = 1;
        CurrentLeadDistance = data_lead.data(j,3);
        CurrentLeadSpeed = data_lead.data(j,4);
        CurrentLeadID = data_lead.data(j,2);
        CurrentLeadTTC = data_lead.data(j,5);
        CurrentLeadNUM = length(CurrentLeadTTC);
        CurrentLeadPosX = 0;
        CurrentLeadPosY = 0;
        CurrentLeadLable = '';
      endif
    endfor

    for j=1:length(timeline_odom)
      if ((timeline_odom(j) >= time_min+i*t_interval) && (timeline_odom(j) < time_min+(i+1)*t_interval))
        UFlag = UFlag + 1;
        hasOd = 1;
        CurrentOdomPosX = data_odom.data(j,2);
        CurrentOdomPosY = data_odom.data(j,3);
        CurrentOdomZ = data_odom.data(j,4);
        CurrentOdomW = data_odom.data(j,5);
        CurrentOdomYaw = asin(CurrentOdomZ)*2;
      endif
    endfor
    
    for j=1:length(timeline_radar)
      if ((timeline_radar(j) >= time_min+i*t_interval) && (timeline_radar(j) < time_min+(i+1)*t_interval))
        UFlag = UFlag + 1;
        hasRa = 1;
        CurrentRadarPosX = data_radar.data(data_radar.data(:,1)==timeline_radar(j),4);
        CurrentRadarPosY = data_radar.data(data_radar.data(:,1)==timeline_radar(j),5);
        CurrentRadarVelX = data_radar.data(data_radar.data(:,1)==timeline_radar(j),6);
        CurrentRadarVelY = data_radar.data(data_radar.data(:,1)==timeline_radar(j),7);
        CurrentRadarID = data_radar.data(data_radar.data(:,1)==timeline_radar(j),2);
        CurrentRadarNUM = length(CurrentRadarID);
      endif
    endfor  
    
    for j=1:length(timeline_l_lane)
      if ((timeline_l_lane(j) >= time_min+i*t_interval) && (timeline_l_lane(j) < time_min+(i+1)*t_interval))
        UFlag = UFlag + 1;
        hasLLa = 1;
        for k=1:((length(data_l_lane.data(j,:))-1)/2)
          if isnan(data_l_lane.data(j,k*2))
            continue
          endif
          CurrentLLanePosX(k) = data_l_lane.data(j,k*2);
          CurrentLLanePosY(k) = data_l_lane.data(j,k*2+1);   
        endfor
      endif
    endfor     
   
    for j=1:length(timeline_r_lane)
      if ((timeline_r_lane(j) >= time_min+i*t_interval) && (timeline_r_lane(j) < time_min+(i+1)*t_interval))
        UFlag = UFlag + 1;
        hasRLa = 1;
        for k=1:((length(data_r_lane.data(j,:))-1)/2)
          if isnan(data_r_lane.data(j,k*2))
            continue
          endif
          CurrentRLanePosX(k) = data_r_lane.data(j,k*2);
          CurrentRLanePosY(k) = data_r_lane.data(j,k*2+1);   
        endfor
      endif
    endfor 
    
    for j=1:length(timeline_l_img)
      if ((timeline_l_img(j) >= time_min+i*t_interval) && (timeline_l_img(j) < time_min+(i+1)*t_interval))
        UFlag = UFlag + 1;
        hasLI = 1;
        LImageTStr = num2str(timeline_l_img(j)/1000);
        LImageName = ['./FrontLeftImg/',LImageTStr(1:10),'.',LImageTStr(11:16),'_L.png'];
        LImage = imread(LImageName);
      endif
    endfor  

    for j=1:length(timeline_r_img)
      if ((timeline_r_img(j) >= time_min+i*t_interval) && (timeline_r_img(j) < time_min+(i+1)*t_interval))
        UFlag = UFlag + 1;
        hasRI = 1;
        RImageTStr = num2str(timeline_r_img(j)/1000);
        RImageName = ['./FrontRightImg/',RImageTStr(1:10),'.',RImageTStr(11:16),'_R.png'];
        RImage = imread(RImageName);
      endif
    endfor  
    
    if((hasFu == 1) && (hasLe == 1) && (hasOd == 1) && (hasRa == 1) && (hasLLa == 1) && (hasRLa == 1) && (hasLI == 1) && (hasRI == 1) &&  (UFlag >= 1))
      figure(i)
      
      subplot(2,2,1)
      image(LImage)
      axis off
      title('Left Camera')
      
      subplot(2,2,[2 4])
      trans_FusionPosX = (CurrentFusionPosX - CurrentOdomPosX) * cos(CurrentOdomYaw) + (CurrentFusionPosY - CurrentOdomPosY) * sin(CurrentOdomYaw);
      trans_FusionPosY =-(CurrentFusionPosX - CurrentOdomPosX) * sin(CurrentOdomYaw) + (CurrentFusionPosY - CurrentOdomPosY) * cos(CurrentOdomYaw);
      trans_LLanePosX = (CurrentLLanePosX - CurrentOdomPosX) * cos(CurrentOdomYaw) + (CurrentLLanePosY - CurrentOdomPosY) * sin(CurrentOdomYaw);
      trans_LLanePosY =-(CurrentLLanePosX - CurrentOdomPosX) * sin(CurrentOdomYaw) + (CurrentLLanePosY - CurrentOdomPosY) * cos(CurrentOdomYaw);   
      trans_RLanePosX = (CurrentRLanePosX - CurrentOdomPosX) * cos(CurrentOdomYaw) + (CurrentRLanePosY - CurrentOdomPosY) * sin(CurrentOdomYaw);
      trans_RLanePosY =-(CurrentRLanePosX - CurrentOdomPosX) * sin(CurrentOdomYaw) + (CurrentRLanePosY - CurrentOdomPosY) * cos(CurrentOdomYaw);      
      if(CurrentLeadID != -1)
        CurrentLeadPosX = trans_FusionPosX(CurrentFusionID == CurrentLeadID);
        CurrentLeadPosY = trans_FusionPosY(CurrentFusionID == CurrentLeadID);
        CurrentLeadLable = ['ID=', num2str(CurrentLeadID);'Dis=',num2str(CurrentLeadDistance);'Spd=',num2str(CurrentLeadSpeed);'TTC=',num2str(CurrentLeadTTC)];
      endif
      plot(-CurrentRadarPosY,CurrentRadarPosX,'b*','MarkerSize',7);
      hold on
      plot(-trans_FusionPosY,trans_FusionPosX,'go','MarkerSize',7);
      if (CurrentLeadID != -1)
        plot(-CurrentLeadPosY,CurrentLeadPosX,'rs','MarkerSize',10);
        annotation('textbox',[0.91,0.85,0.5,0.6],'String', CurrentLeadLable);
      endif
      plot(-trans_LLanePosY,trans_LLanePosX,'k.','MarkerSize',4);
      plot(-trans_RLanePosY,trans_RLanePosX,'k.','MarkerSize',4);
      legend('Radar','Fused','Lead')    
      set(gca,'XLim',[-30 30],'YLim',[-10 200]);
      
      subplot(2,2,3)
      image(RImage)
      axis off
      title('Right Camera')
      
      plotname = ['./PlotOutput/',num2str(filenum),'_' ,num2str((time_min+i*t_interval)/1000),'.jpg'];
      saveas(gcf,plotname);
      
      kfinish = 1;
      close
    endif
    
    UFlag = 0;
  endfor
  
  clearvars -except Path BagFiles filenum
endfor
cd ..