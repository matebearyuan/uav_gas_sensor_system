clc
clear all
filePath = fullfile('C:\Users\hw\Desktop\workspace', 'circle_twodrones_1hz.bag');
bagselect = rosbag(filePath);
%selectOptions = {'MessageType', {'/firefly/Concentration'}};
bagselect1 = select(bagselect, 'topic','/fireflymaster/gas1_concentration');
bagselect2 = select(bagselect, 'topic','/fireflymaster/gas2_concentration');
bagselect3 = select(bagselect, 'topic','/fireflymaster/gas3_concentration');
ts1 = timeseries(bagselect1);
ts2 = timeseries(bagselect2);
ts3 = timeseries(bagselect3);
bagselect5 = select(bagselect, 'topic','/fireflyslave/gas1_concentration');
bagselect6 = select(bagselect, 'topic','/fireflyslave/gas2_concentration');
bagselect7 = select(bagselect, 'topic','/fireflyslave/gas3_concentration');
ts5 = timeseries(bagselect5);
ts6 = timeseries(bagselect6);
ts7 = timeseries(bagselect7);


bagselect4 = select(bagselect, 'topic','/fireflymaster/odometry_sensor1/pose');
bagselect8 = select(bagselect, 'topic','/fireflyslave/odometry_sensor1/pose');
ts4 = timeseries(bagselect4);
ts8 = timeseries(bagselect8);

figure(2)
plot3(ts4.Data(:,1),ts4.Data(:,2),ts4.Data(:,3), 'LineWidth', 3);
hold on
plot3(ts8.Data(:,1),ts8.Data(:,2),ts8.Data(:,3), 'LineWidth', 3);
hold off


x_m = round(max(max(ts4.Data(:,1)),max(ts8.Data(:,1)))*10); %dm
x_n = round(min(min(ts4.Data(:,1)),min(ts8.Data(:,1)))*10); %dm


y_m = round(max(max(ts4.Data(:,2)),max(ts8.Data(:,2)))*10); %dm
y_n = round(min(min(ts4.Data(:,2)),min(ts8.Data(:,2)))*10); %dm
x = x_n:1:x_m;
y = y_n:1:y_m;

Map1 = ones(size(y,2),size(x,2))*(-1);
Map2 = ones(size(y,2),size(x,2))*(-1);
Map3 = ones(size(y,2),size(x,2))*(-1);

for i = 1:size(ts1.time)
  t=ts1.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     l=round(ts4.Data(r(1),1)*10)+1-x_n;
     k=round(ts4.Data(r(1),2)*10)+1-y_n;
     Map1(k,l)=ts1.Data(i,1);
  elseif size(r,1)==1
     l=round(ts4.Data(r,1)*10)+1-x_n;
     k=round(ts4.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts1.Data(i,1);

  end
end
for i = 1:size(ts5.time)
  t=ts5.time(i);
  r = find(ts8.time==t);
  if size(r,1)>1
     l=round(ts8.Data(r(1),1)*10)+1-x_n;
     k=round(ts8.Data(r(1),2)*10)+1-y_n;
     Map1(k,l)=ts5.Data(i,1);

  elseif size(r,1)==1
     l=round(ts8.Data(r,1)*10)+1-x_n;
     k=round(ts8.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts5.Data(i,1);
  end
end
figure(4)
surf(x,y,Map1);
axis([min(x),max(x),min(y),max(y)]);

for i = 1:size(ts2.time)
  t=ts2.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     l=round(ts4.Data(r(1),1)*10)+1-x_n;
     k=round(ts4.Data(r(1),2)*10)+1-y_n;
     Map2(k,l)=ts2.Data(i,1);
  elseif size(r,1)==1
     l=round(ts4.Data(r,1)*10)+1-x_n;
     k=round(ts4.Data(r,2)*10)+1-y_n;
     Map2(k,l)=ts2.Data(i,1);
  end
end
for i = 1:size(ts6.time)
  t=ts6.time(i);
  r = find(ts8.time==t);
  if size(r,1)>1
     l=round(ts8.Data(r(1),1)*10)+1-x_n;
     k=round(ts8.Data(r(1),2)*10)+1-y_n;
     Map2(k,l)=ts6.Data(i,1);
  elseif size(r,1)==1
     l=round(ts8.Data(r,1)*10)+1-x_n;
     k=round(ts8.Data(r,2)*10)+1-y_n;
     Map2(k,l)=ts6.Data(i,1);
  end
end
figure(5)
surf(x,y,Map2);
axis([min(x),max(x),min(y),max(y)]);

for i = 1:size(ts3.time)
  t=ts3.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     l=round(ts4.Data(r(1),1)*10)+1-x_n;
     k=round(ts4.Data(r(1),2)*10)+1-y_n;
     Map3(k,l)=ts3.Data(i,1);
  elseif size(r,1)==1
     l=round(ts4.Data(r,1)*10)+1-x_n;
     k=round(ts4.Data(r,2)*10)+1-y_n;
     Map3(k,l)=ts3.Data(i,1);
  end
end
for i = 1:size(ts7.time)
  t=ts7.time(i);
  r = find(ts8.time==t);
  if size(r,1)>1
     l=round(ts8.Data(r(1),1)*10)+1-x_n;
     k=round(ts8.Data(r(1),2)*10)+1-y_n;
     Map3(k,l)=ts7.Data(i,1);
  elseif size(r,1)==1
     l=round(ts8.Data(r,1)*10)+1-x_n;
     k=round(ts8.Data(r,2)*10)+1-y_n;
     Map3(k,l)=ts7.Data(i,1);
  end
end
figure(6)
surf(x,y,Map3);
axis([min(x),max(x),min(y),max(y)]);
figure(7)
surf(x,y,Map1);
axis([min(x),max(x),min(y),max(y)]);
hold on 
surf(x,y,Map2);
surf(x,y,Map3);
hold off