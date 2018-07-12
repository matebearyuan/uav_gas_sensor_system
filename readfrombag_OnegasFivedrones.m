clc
clear all
filePath = fullfile('C:\Users\hw\Desktop\workspace', 'tracking_fivedrones_1p0hz_master_0x5x6p5x4.bag');
bagselect = rosbag(filePath);
%selectOptions = {'MessageType', {'/firefly/Concentration'}};
bagselect1 = select(bagselect, 'topic','/firefly1/gas1_concentration');
bagselect2 = select(bagselect, 'topic','/firefly2/gas1_concentration');
bagselect3 = select(bagselect, 'topic','/firefly3/gas1_concentration');
bagselect9 = select(bagselect, 'topic','/firefly4/gas1_concentration');
bagselect10 = select(bagselect, 'topic','/firefly5/gas1_concentration');
ts1 = timeseries(bagselect1);
ts2 = timeseries(bagselect2);
ts3 = timeseries(bagselect3);
ts9 = timeseries(bagselect9);
ts10 = timeseries(bagselect10);

bagselect4 = select(bagselect, 'topic','/firefly1/odometry_sensor1/pose');
bagselect5 = select(bagselect, 'topic','/firefly2/odometry_sensor1/pose');
bagselect6 = select(bagselect, 'topic','/firefly3/odometry_sensor1/pose');
bagselect7 = select(bagselect, 'topic','/firefly4/odometry_sensor1/pose');
bagselect8 = select(bagselect, 'topic','/firefly5/odometry_sensor1/pose');
ts4 = timeseries(bagselect4);
ts5 = timeseries(bagselect5);
ts6 = timeseries(bagselect6);
ts7 = timeseries(bagselect7);
ts8 = timeseries(bagselect8);
figure(1)
plot(ts1.Time,ts1.Data(:,1), 'LineWidth', 2);
hold on
grid on
plot(ts2.Time,ts2.Data(:,1), 'LineWidth', 2);
plot(ts3.Time,ts3.Data(:,1), 'LineWidth', 2);
plot(ts9.Time,ts9.Data(:,1), 'LineWidth', 2);
plot(ts10.Time,ts10.Data(:,1), 'LineWidth', 2);
hold off
figure(2)
plot(ts4.Time,ts4.Data(:,1), 'LineWidth', 2);
hold on
grid on
plot(ts5.Time,ts5.Data(:,1), 'LineWidth', 2);
plot(ts6.Time,ts6.Data(:,1), 'LineWidth', 2);
plot(ts7.Time,ts7.Data(:,1), 'LineWidth', 2);
plot(ts8.Time,ts8.Data(:,1), 'LineWidth', 2);
hold off

for i = 1:size(ts1.time)
  t=ts1.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     T1(i,1)=round(ts4.Data(r(1),1)*10);
     T1(i,2)=round(ts4.Data(r(1),2)*10);
     T1(i,3)=ts1.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     T1(i,1)=round(ts4.Data(r(1),1)*10);
     T1(i,2)=round(ts4.Data(r(1),2)*10);
     T1(i,3)=ts1.Data(i,1)+randn*0.03;
  end
end
for i = 1:size(ts2.time)
  t=ts2.time(i);
  r = find(ts5.time==t);
  if size(r,1)>1
     T2(i,1)=round(ts5.Data(r(1),1)*10);
     T2(i,2)=round(ts5.Data(r(1),2)*10);
     T2(i,3)=ts2.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     T2(i,1)=round(ts5.Data(r(1),1)*10);
     T2(i,2)=round(ts5.Data(r(1),2)*10);
     T2(i,3)=ts2.Data(i,1)+randn*0.03;
  end
end
for i = 1:size(ts3.time)
  t=ts3.time(i);
  r = find(ts6.time==t);
  if size(r,1)>1
     T3(i,1)=round(ts6.Data(r(1),1)*10);
     T3(i,2)=round(ts6.Data(r(1),2)*10);
     T3(i,3)=ts3.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     T3(i,1)=round(ts6.Data(r(1),1)*10);
     T3(i,2)=round(ts6.Data(r(1),2)*10);
     T3(i,3)=ts3.Data(i,1)+randn*0.03;
  end
end
for i = 1:size(ts9.time)
  t=ts9.time(i);
  r = find(ts7.time==t);
  if size(r,1)>1
     T4(i,1)=round(ts7.Data(r(1),1)*10);
     T4(i,2)=round(ts7.Data(r(1),2)*10);
     T4(i,3)=ts9.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     T4(i,1)=round(ts7.Data(r(1),1)*10);
     T4(i,2)=round(ts7.Data(r(1),2)*10);
     T4(i,3)=ts9.Data(i,1)+randn*0.03;
  end
end
for i = 1:size(ts10.time)
  t=ts10.time(i);
  r = find(ts8.time==t);
  if size(r,1)>1
     T5(i,1)=round(ts8.Data(r(1),1)*10);
     T5(i,2)=round(ts8.Data(r(1),2)*10);
     T5(i,3)=ts10.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     T5(i,1)=round(ts8.Data(r(1),1)*10);
     T5(i,2)=round(ts8.Data(r(1),2)*10);
     T5(i,3)=ts10.Data(i,1)+randn*0.03;
  end
end
figure(3)
plot3(T1(:,1),T1(:,2),T1(:,3), 'LineWidth', 3);
hold on
plot3(T2(:,1),T2(:,2),T2(:,3),  'LineWidth', 3);
plot3(T3(:,1),T3(:,2),T3(:,3),  'LineWidth', 3);
plot3(T4(:,1),T4(:,2),T4(:,3),  'LineWidth', 3);
plot3(T5(:,1),T5(:,2),T5(:,3), 'LineWidth', 3);
axis([-300,250,-200,200,0,1]);
hold off


x_m = max(max(max(ts4.Data(:,1)),max(ts5.Data(:,1))), max(ts6.Data(:,1))); %dm
x_m = max(x_m,max(ts7.Data(:,1)));
x_m = max(x_m,max(ts8.Data(:,1)));
x_m = round(x_m*10);

x_n = min(min(min(ts4.Data(:,1)),min(ts5.Data(:,1))), min(ts6.Data(:,1))); %dm
x_n = min(x_n,min(ts7.Data(:,1)));
x_n = min(x_n,min(ts8.Data(:,1)));
x_n = round(x_n*10);

y_m = max(max(max(ts4.Data(:,2)),max(ts5.Data(:,2))), max(ts6.Data(:,2))); %dm
y_m = max(y_m,max(ts7.Data(:,2)));
y_m = max(y_m,max(ts8.Data(:,2)));
y_m = round(y_m*10);
y_n = min(min(min(ts4.Data(:,2)),min(ts5.Data(:,2))), min(ts6.Data(:,2))); %dm
y_n = min(y_n,min(ts7.Data(:,2)));
y_n = min(y_n,min(ts8.Data(:,2)));
y_n = round(y_n*10);
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

% add the four drone
for i = 1:size(ts9.time)
  t=ts9.time(i);
  r = find(ts6.time==t);
  if size(r,1)>1
     l=round(ts7.Data(r(1),1)*10)+1-x_n;
     k=round(ts7.Data(r(1),2)*10)+1-y_n;
     Map1(k,l)=ts9.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     l=round(ts7.Data(r,1)*10)+1-x_n;
     k=round(ts7.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts9.Data(i,1)+randn*0.03;
  end
end

% add the five drone
for i = 1:size(ts10.time)
  t=ts10.time(i);
  r = find(ts8.time==t);
  if size(r,1)>1
     l=round(ts8.Data(r(1),1)*10)+1-x_n;
     k=round(ts8.Data(r(1),2)*10)+1-y_n;
     Map1(k,l)=ts10.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     l=round(ts8.Data(r,1)*10)+1-x_n;
     k=round(ts8.Data(r,2)*10)+1-y_n;
     Map1(k,l)=ts10.Data(i,1)+randn*0.03;
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
plot3(ts7.Data(:,1)*10,ts7.Data(:,2)*10,ts7.Data(:,3)*10, 'LineWidth', 3);
plot3(ts8.Data(:,1)*10,ts8.Data(:,2)*10,ts8.Data(:,3)*10, 'LineWidth', 3);
axis([min(x),max(x),min(y),max(y)]);
hold off

F=[0,-3,6.5,4];
e = abs(F - fitresult1/10)
e_mean = sum(abs(F - fitresult1/10))/4
