clc
clear all
filePath = fullfile('C:\Users\hw\Desktop\workspace', 'tracking_threedrones_1p0hz_master.bag');
bagselect = rosbag(filePath);
%selectOptions = {'MessageType', {'/firefly/Concentration'}};
bagselect1 = select(bagselect, 'topic','/firefly1/gas1_concentration');
bagselect2 = select(bagselect, 'topic','/firefly2/gas1_concentration');
bagselect3 = select(bagselect, 'topic','/firefly3/gas1_concentration');
ts1 = timeseries(bagselect1);
ts2 = timeseries(bagselect2);
ts3 = timeseries(bagselect3);

bagselect4 = select(bagselect, 'topic','/firefly1/odometry_sensor1/pose');
bagselect5 = select(bagselect, 'topic','/firefly2/odometry_sensor1/pose');
bagselect6 = select(bagselect, 'topic','/firefly3/odometry_sensor1/pose');
ts4 = timeseries(bagselect4);
ts5 = timeseries(bagselect5);
ts6 = timeseries(bagselect6);
figure(1)
plot(ts1.Time,ts1.Data(:,1), 'LineWidth', 2);
hold on
grid on
plot(ts2.Time,ts2.Data(:,1), 'LineWidth', 2);
plot(ts3.Time,ts3.Data(:,1), 'LineWidth', 2);
hold off
figure(2)
plot(ts4.Time,ts4.Data(:,1), 'LineWidth', 2);
hold on
grid on
plot(ts5.Time,ts5.Data(:,1), 'LineWidth', 2);
plot(ts6.Time,ts6.Data(:,1), 'LineWidth', 2);
hold off
figure(3)
plot3(ts4.Data(:,1),ts4.Data(:,2),ts4.Data(:,3), 'LineWidth', 3);
hold on
plot3(ts5.Data(:,1),ts5.Data(:,2),ts5.Data(:,3), 'LineWidth', 3);
plot3(ts6.Data(:,1),ts6.Data(:,2),ts6.Data(:,3), 'LineWidth', 3);
hold off


x_m = round(max(max(max(ts4.Data(:,1)),max(ts5.Data(:,1))), max(ts6.Data(:,1)))*10); %dm
x_n = round(min(min(min(ts4.Data(:,1)),min(ts5.Data(:,1))), min(ts6.Data(:,1)))*10); %dm

y_m = round(max(max(max(ts4.Data(:,2)),max(ts5.Data(:,2))), max(ts6.Data(:,2)))*10); %dm
y_n = round(min(min(min(ts4.Data(:,2)),min(ts5.Data(:,2))), min(ts6.Data(:,2)))*10); %dm
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
     Map1(k,l)=ts1.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     l=round(ts4.Data(r,1)*10)+1-x_n;
     k=round(ts4.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts1.Data(i,1)+randn*0.03;
  end
end

% add the sencond drone
for i = 1:size(ts2.time)
  t=ts2.time(i);
  r = find(ts5.time==t);
  if size(r,1)>1
     l=round(ts5.Data(r(1),1)*10)+1-x_n;
     k=round(ts5.Data(r(1),2)*10)+1-y_n;
     Map1(k,l)=ts2.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     l=round(ts5.Data(r,1)*10)+1-x_n;
     k=round(ts5.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts2.Data(i,1)+randn*0.03;
  end
end

% add the third drone
for i = 1:size(ts3.time)
  t=ts3.time(i);
  r = find(ts6.time==t);
  if size(r,1)>1
     l=round(ts6.Data(r(1),1)*10)+1-x_n;
     k=round(ts6.Data(r(1),2)*10)+1-y_n;
     Map1(k,l)=ts3.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     l=round(ts6.Data(r,1)*10)+1-x_n;
     k=round(ts6.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts3.Data(i,1)+randn*0.03;
  end
end

fitresult1 = Fitting(x, y, Map1)
figure(7)
surf(x,y,Map1);
axis([min(x),max(x),min(y),max(y)]);
a = fitresult1(1);b=fitresult1(2);c=fitresult1(3);d=fitresult1(4);
for i = 1:size(x,2)
    for j = 1:size(y,2)
        f1(i,j) = exp(-(x(i)-a)^2/(2*c*c)-(y(j)-b)^2/(2*d*d));
    end
end
figure(4)
surf(x,y,f1');
hold on
plot3(ts4.Data(:,1)*10,ts4.Data(:,2)*10,ts4.Data(:,3)*10, 'LineWidth', 3);
plot3(ts5.Data(:,1)*10,ts5.Data(:,2)*10,ts5.Data(:,3)*10, 'LineWidth', 3);
plot3(ts6.Data(:,1)*10,ts6.Data(:,2)*10,ts6.Data(:,3)*10, 'LineWidth', 3);
axis([min(x),max(x),min(y),max(y)]);
hold off

F=[6,8,4.5,8];
e = abs(F - fitresult1/10)
e_mean = sum(abs(F - fitresult1/10))/4
