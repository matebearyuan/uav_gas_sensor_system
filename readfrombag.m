clc
clear all
filePath = fullfile('C:\Users\hw\Desktop\workspace', 'spindle_onedrone_1p0hz.bag');
bagselect = rosbag(filePath);
%selectOptions = {'MessageType', {'/firefly/Concentration'}};
bagselect2 = select(bagselect, 'topic','/firefly/odometry_sensor1/pose');
ts2 = timeseries(bagselect2);
figure(2)
plot3(ts2.Data(:,1),ts2.Data(:,2),ts2.Data(:,3), 'LineWidth', 3);

bagselect1 = select(bagselect, 'topic','/firefly/gas1_concentration');

bagselect3 = select(bagselect, 'topic','/firefly/gas2_concentration');
bagselect4 = select(bagselect, 'topic','/firefly/gas3_concentration');

ts1 = timeseries(bagselect1);
ts3 = timeseries(bagselect3);
ts4 = timeseries(bagselect4);
figure(1)
plot(ts1.Time,ts1.Data(:,1), 'LineWidth', 3);


x_p = round(max(ts2.Data(:,1))*10); %dm
x_n = round(min(ts2.Data(:,1))*10); %dm

x = x_n:1:x_p;
y_p = round(max(ts2.Data(:,2))*10); %dm
y_n = round(min(ts2.Data(:,2))*10); %dm

y = y_n:1:y_p;

Map1 = ones(size(y,2),size(x,2))*(-1);
Map2 = ones(size(y,2),size(x,2))*(-1);
Map3 = ones(size(y,2),size(x,2))*(-1);

for i = 1:size(ts1.time)
  t=ts1.time(i);
  r = find(ts2.time==t);
  if size(r,1)>1
     l=round(ts2.Data(r(1),1)*10)+1-x_n;
     k=round(ts2.Data(r(1),2)*10)+1-y_n;
     Map1(k,l)=ts1.Data(i,1);
  elseif size(r,1)==1
     l=round(ts2.Data(r,1)*10)+1-x_n;
     k=round(ts2.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts1.Data(i,1);
  end
end

for i = 1:size(ts3.time)
  t=ts3.time(i);
  r = find(ts2.time==t);
  if size(r,1)>1
     l=round(ts2.Data(r(1),1)*10)+1-x_n;
     k=round(ts2.Data(r(1),2)*10)+1-y_n;
     Map2(k,l)=ts3.Data(i,1);
  elseif size(r,1)==1
     l=round(ts2.Data(r,1)*10)+1-x_n;
     k=round(ts2.Data(r,2)*10)+1-y_n;
     Map2(k,l)=ts3.Data(i,1);
  end
end
for i = 1:size(ts4.time)
  t=ts4.time(i);
  r = find(ts2.time==t);
  if size(r,1)>1
     l=round(ts2.Data(r(1),1)*10)+1-x_n;
     k=round(ts2.Data(r(1),2)*10)+1-y_n;
     Map3(k,l)=ts4.Data(i,1);
  elseif size(r,1)==1
     l=round(ts2.Data(r,1)*10)+1-x_n;
     k=round(ts2.Data(r,2)*10)+1-y_n;
     Map3(k,l)=ts4.Data(i,1);
  end
end
figure(3)
surf(x,y,Map1)
hold on 
surf(x,y,Map2)
surf(x,y,Map3)
hold off

figure(4)
plot(ts2.Time, ts2.Data(:,1));
hold on
plot(ts2.Time, ts2.Data(:,2));
plot(ts2.Time, ts2.Data(:,3));
grid
hold off

      