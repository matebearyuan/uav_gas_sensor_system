clc
clear all
filePath = fullfile('C:\Users\hw\Desktop\workspace\bagfiles', 'rosbag_demo_p3p.bag');
bagselect = rosbag(filePath);
% bagselect1 = select(bagselect, 'topic','/firefly/odometry_sensor1/pose');
% t1=timeseries(bagselect1);
bagselect2 = select(bagselect, 'topic','/ground_camera_1/P3P/estimated_pose');
t2=timeseries(bagselect2);
t1=readMessages(bagselect2);
for (i=1:size(t1,1))
    pose_cov(i,1:36) = (t1{i,1}.Pose.Covariance)';
    position_cov(i,1) = pose_cov(i,1)+pose_cov(i,8)+pose_cov(i,15);
    orientation_cov(i,1) = pose_cov(i,22)+pose_cov(i,29)+pose_cov(i,36);
end
x = 1:size(position_cov,1);
figure(3)
hold on
grid on
% plot3(ground_Pose(:,2),ground_Pose(:,3),ground_Pose(:,4),'-', 'LineWidth', 1.5);
[AX,H1,H2]=plotyy(x,position_cov,x,orientation_cov,@plot);
set(get(AX(1),'ylabel'),'string', 'position error (m)','fontsize',18);
set(get(AX(2),'ylabel'),'string', 'orientation error (rad)','fontsize',18);
xlabel('frame number','fontsize',18);
set(H1,'Linestyle','-');
set(H2,'Linestyle','-');
set(gcf,'color','white')
set(gca,'linewidth',1.5) %??????
hold off
% ground_Pose(:,2:4) = t1.Data(:,1:3);
% ground_Pose(:,1) = t1.Time;
%B = [0.48,0.48,0];
P3P_Pose(:,2:4) = t2.Data(:,4:6); %+ repmat(B, size(t2.Time,1), 1);
P3P_Pose(:,1) = t2.Time;
figure(1)
% plot(t1.Time,t1.Data(:,1),'-', 'LineWidth', 1.5);
hold on
grid on
% plot(t1.Time,t1.Data(:,2),'-',  'LineWidth', 1.5);
% plot(t1.Time,t1.Data(:,3),'-', 'LineWidth', 1.5);
plot(P3P_Pose(:,2),'-', 'LineWidth', 1.5);
plot(P3P_Pose(:,3),'-', 'LineWidth', 1.5);
plot(P3P_Pose(:,4),'-', 'LineWidth', 1.5);
hold off
% for (i=1:size(t1.Data,1))
%     q=t1.Data(i,4:7);
%     [R,ground_Pose(i,5:7)]=setRotation(q);
% end
for (i=1:size(t2.Data,1))
    q=t2.Data(i,7:10);
    [R, P3P_Pose(i,5:7)]=setRotation(q);
end
figure(2)
% plot(t1.Time,ground_Pose(:,5),'-', 'LineWidth', 1.5);
hold on
grid on
% plot(t1.Time,ground_Pose(:,6),'-',  'LineWidth', 1.5);
% plot(t1.Time,ground_Pose(:,7),'-', 'LineWidth', 1.5);
plot(P3P_Pose(:,5),'-','markersize', 2, 'LineWidth', 1.5);
plot(P3P_Pose(:,6),'-', 'markersize', 2,'LineWidth', 1.5);
plot(P3P_Pose(:,7),'-','markersize', 2, 'LineWidth', 1.5);
hold off
% error = zeros(size(P3P_Pose,1),6);
% for(i=1:size(P3P_Pose,1))
%   t=P3P_Pose(i,1);
%   r = find(ground_Pose(:,1)==t);
%   if size(r,1)>0
%       error(i,1:6) = (ground_Pose(r(1),2:7)-P3P_Pose(i,2:7));
%   else
%       error(i,1:6)=zeros(1,6);
%   end
% end
% figure(3)
% plot(P3P_Pose(:,1),error(:,1),'-', 'LineWidth', 1.5);
% hold on
% grid on
% plot(P3P_Pose(:,1),error(:,2),'-',  'LineWidth', 1.5);
% plot(P3P_Pose(:,1),error(:,3),'-', 'LineWidth', 1.5);
% hold off
% figure(4)
% plot(P3P_Pose(:,1),error(:,4),'-', 'LineWidth', 1.5);
% hold on
% grid on
% plot(P3P_Pose(:,1),error(:,5),'-',  'LineWidth', 1.5);
% plot(P3P_Pose(:,1),error(:,6),'-', 'LineWidth', 1.5);
% hold off
figure(5)
hold on
grid on
% plot3(ground_Pose(:,2),ground_Pose(:,3),ground_Pose(:,4),'-', 'LineWidth', 1.5);
plot3(P3P_Pose(:,2),P3P_Pose(:,3),P3P_Pose(:,4),'-k', 'LineWidth', 1.5);
hold off