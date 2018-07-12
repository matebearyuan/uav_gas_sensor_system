clc
clear all
filePath = fullfile('C:\Users\hw\Desktop\workspace', 'spindle_onedrone_1p0hz.bag');
bagselect = rosbag(filePath);
%selectOptions = {'MessageType', {'/firefly/Concentration'}};
bagselect1 = select(bagselect, 'topic','/firefly/gas1_concentration');
bagselect2 = select(bagselect, 'topic','/firefly/gas2_concentration');
bagselect3 = select(bagselect, 'topic','/firefly/gas3_concentration');
ts1 = timeseries(bagselect1);
ts2 = timeseries(bagselect2);
ts3 = timeseries(bagselect3);
figure(1)
plot(ts1.Time,ts1.Data(:,1), 'LineWidth', 3);
hold on
plot(ts2.Time,ts2.Data(:,1), 'LineWidth', 3);
plot(ts3.Time,ts3.Data(:,1), 'LineWidth', 3);
hold off

bagselect4 = select(bagselect, 'topic','/firefly/odometry_sensor1/pose');

ts4 = timeseries(bagselect4);


figure(2)
plot3(ts4.Data(:,1),ts4.Data(:,2),ts4.Data(:,3), 'LineWidth', 3);
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
  r = find(ts4.time==t);
  if size(r,1)>1
     T2(i,1)=round(ts4.Data(r(1),1)*10);
     T2(i,2)=round(ts4.Data(r(1),2)*10);
     T2(i,3)=ts2.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     T2(i,1)=round(ts4.Data(r(1),1)*10);
     T2(i,2)=round(ts4.Data(r(1),2)*10);
     T2(i,3)=ts2.Data(i,1)+randn*0.03;
  end
end
for i = 1:size(ts3.time)
  t=ts3.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     T3(i,1)=round(ts4.Data(r(1),1)*10);
     T3(i,2)=round(ts4.Data(r(1),2)*10);
     T3(i,3)=ts3.Data(i,1)+randn*0.03;
  elseif size(r,1)==1
     T3(i,1)=round(ts4.Data(r(1),1)*10);
     T3(i,2)=round(ts4.Data(r(1),2)*10);
     T3(i,3)=ts3.Data(i,1)+randn*0.03;
  end
end

figure(7)
plot3(T1(:,1),T1(:,2),T1(:,3), '--*','markersize',5,'LineWidth', 2);
hold on
plot3(T2(:,1),T2(:,2),T2(:,3),'--*','markersize',5, 'LineWidth', 2);
plot3(T3(:,1),T3(:,2),T3(:,3), '--*', 'markersize',5, 'LineWidth', 2);
axis([-100,100,-100,100,0,1]);
grid on
hold off

m_p = round(max(ts4.Data(:,1))*10); %dm
m_n = round(min(ts4.Data(:,1))*10); %dm


n_p = round(max(ts4.Data(:,2))*10); %dm
n_n = round(min(ts4.Data(:,2))*10); %dm
x = m_n:1:m_p;
y = n_n:1:n_p;

Map1 = zeros(size(y,2),size(x,2))*(-1);
Map2 = zeros(size(y,2),size(x,2))*(-1);
Map3 = zeros(size(y,2),size(x,2))*(-1);

for i = 1:size(ts1.time)
  t=ts1.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     l=round(ts4.Data(r(1),1)*10)+1+abs(m_n);
     k=round(ts4.Data(r(1),2)*10)+1+abs(n_n);
     Map1(k,l)=ts1.Data(i,1);
  elseif size(r,1)==1
     l=round(ts4.Data(r,1)*10)+1+abs(m_n);
     k=round(ts4.Data(r,2)*10)+1+abs(n_n);
     Map1(k,l)=ts1.Data(i,1);
  end
end
for i = 1:size(ts2.time)
  t=ts2.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     l=round(ts4.Data(r(1),1)*10)+1+abs(m_n);
     k=round(ts4.Data(r(1),2)*10)+1+abs(n_n);
     Map2(k,l)=ts2.Data(i,1);

  elseif size(r,1)==1
     l=round(ts4.Data(r,1)*10)+1+abs(m_n);
     k=round(ts4.Data(r,2)*10)+1+abs(n_n);
     Map2(k,l)=ts2.Data(i,1);

  end
end
for i = 1:size(ts3.time)
  t=ts1.time(i);
  r = find(ts4.time==t);
  if size(r,1)>1
     l=round(ts4.Data(r(1),1)*10)+1+abs(m_n);
     k=round(ts4.Data(r(1),2)*10)+1+abs(n_n);
     Map3(k,l)=ts3.Data(i,1);

  elseif size(r,1)==1
     l=round(ts4.Data(r,1)*10)+1+abs(m_n);
     k=round(ts4.Data(r,2)*10)+1+abs(n_n);
     Map3(k,l)=ts3.Data(i,1);
  end
end
figure(3)
surf(x,y,Map1);
axis([min(x),max(x),min(y),max(y),0,1]);
hold on 
surf(x,y,Map2);
surf(x,y,Map3);
hold off

fitresult1 = Fitting(x, y, Map1);
fitresult2 = Fitting(x, y, Map2);
fitresult3 = Fitting(x, y, Map3);
fitresult = [fitresult1;fitresult2;fitresult3]
[xData, yData, zData] = prepareSurfaceData( x, y, Map1);
ft = fittype( 'exp(-(x-a)^2/(2*c*c)-(y-b)^2/(2*d*d))', 'independent', {'x', 'y'}, 'dependent', 'z' );
ft1 = sfit(ft,fitresult1(1),fitresult1(2),fitresult1(3),fitresult1(4));
ft2 = sfit(ft,fitresult2(1),fitresult2(2),fitresult2(3),fitresult2(4));
ft3 = sfit(ft,fitresult3(1),fitresult3(2),fitresult3(3),fitresult3(4));
figure(4)
h1 = plot(ft1,[xData, yData], zData);
axis([min(x),max(x),min(y),max(y),0,1]);
%legend( h1, 'gas distribution fitting', 'Map1 vs. x, y', 'Location', 'NorthEast' );
hold on
[xData, yData, zData] = prepareSurfaceData( x, y, Map2);
h2 = plot(ft2,[xData, yData], zData);
[xData, yData, zData] = prepareSurfaceData( x, y, Map3);
h3 = plot(ft3,[xData, yData], zData);
hold off
a = fitresult1(1);b=fitresult1(2);c=fitresult1(3);d=fitresult1(4);
for i = 1:size(x,2)
    for j = 1:size(y,2)
        f1(i,j) = exp(-(x(i)-a)^2/(2*c*c)-(y(j)-b)^2/(2*d*d));
    end
end
a = fitresult2(1);b=fitresult2(2);c=fitresult2(3);d=fitresult2(4);
for i = 1:size(x,2)
    for j = 1:size(y,2)
        f2(i,j) = exp(-(x(i)-a)^2/(2*c*c)-(y(j)-b)^2/(2*d*d));
    end
end
a = fitresult3(1);b=fitresult3(2);c=fitresult3(3);d=fitresult3(4);
for i = 1:size(x,2)
    for j = 1:size(y,2)
        f3(i,j) = exp(-(x(i)-a)^2/(2*c*c)-(y(j)-b)^2/(2*d*d));
    end
end
figure(5)
surf(x,y,f1');

hold on
axis([min(x),max(x),min(y),max(y)]);
surf(x,y,f2');
surf(x,y,f3');
plot3(ts4.Data(:,1)*10,ts4.Data(:,2)*10,ts4.Data(:,3)*10,'r-', 'LineWidth', 3);
hold off     