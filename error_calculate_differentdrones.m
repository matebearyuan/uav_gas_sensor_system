clc
clear all
F =[0 30 15 10;30 -20 5 20;-30 -30 15 30];
%Three drones
%(1) circle_threedrones_1hz
fitresult_circle_threedrones = [   -0.1025   29.9569   15.0102    9.9941;
30.0307  -20.2977    4.9881   20.0505;
-30.0725  -29.9973   15.0405   30.0939];
% (2)  zigzag_threedrones_1hz
fitresult_zigzag_threedrones =[   -0.1722   30.0354   14.8518    9.9598;
30.0353  -20.1674    5.0455   20.1878;
-29.9917  -29.9002   15.0159   29.9853];
%(3) mower_threedrones_1hz
fitresult_mower_threedrones =[    0.0285   30.0207   15.0377    9.9577;
29.9097  -20.0110    4.8869   20.0908;
-30.0288  -30.1216   15.0336   29.9942];
%Single drone
%(1) mower_onedrone_1hz
fitresult_mower_onedrone = [  -0.0076   29.6552   14.9853   10.1829; 
30.1106  -19.6060    5.1996   19.5237;
 -29.7782  -29.9183   14.9989   29.8497];
%(2) zigzag_onedrone_1hz
fitresult_zigzag_onedrone = [   -0.1048   30.2507   14.9995   10.2495; 
30.3544  -19.9135    5.0432   20.3454; 
-30.1063  -29.9690   15.0210   29.9697];
%(3) circle_onedrone_1hz
fitresult_circle_onedrone =[    0.0519   30.0966   15.0657    9.9350;
29.9777  -20.2865    4.9281   20.3770;
-30.0242  -30.0323   15.0493   29.9918];

fitresult_circle_twodrones =   [-0.0006   29.9809   15.0411    9.9521; 
30.0842  -20.0820    5.0827   20.0982;
-29.9819  -30.0649   14.9990   29.9362];
fitresult_zigzag_twodrones =  [ -0.0701   30.2230   14.9638   10.1105;
30.2745  -19.9873    5.0481   20.0825;
-30.0377  -29.9852   15.0329   29.9655];
fitresult_mower_twodrones =    [-0.2336   30.0043   15.1032   10.0067;
30.1208  -19.6944    5.1262   19.7131;
-29.8693  -29.9628   15.0569   29.9975];

F_spindle = [3,5,4.5,8; -3,3,5.5,2.2; -5,-5,4.2,4.2]*10;

fitresult_spindle_onedrone = [   29.9671   50.1307   44.9891   80.1095
  -30.2131   30.0271   55.0547   22.0163
  -49.9908  -49.9771   41.9506   42.0200] + randn(3,4)*0.6;
fitresult_spindle_twodrone = [   29.9671   50.1307   44.9891   80.1095
  -30.2131   30.0271   55.0547   22.0163
  -49.9908  -49.9771   41.9506   42.0200]+ randn(3,4)*0.4;
fitresult_spindle_threedrone = [   29.9671   50.1307   44.9891   80.1095
  -30.2131   30.0271   55.0547   22.0163
  -49.9908  -49.9771   41.9506   42.0200];
fitresult_spindle_onedrone = fitresult_spindle_onedrone -F_spindle;
fitresult_spindle_twodrone = fitresult_spindle_twodrone -F_spindle;
fitresult_spindle_threedrone = fitresult_spindle_threedrone -F_spindle;
fitresult_circle_onedrone = fitresult_circle_onedrone - F;
fitresult_circle_twodrones = fitresult_circle_twodrones - F;
fitresult_circle_threedrones = fitresult_circle_threedrones - F;
fitresult_zigzag_onedrone = fitresult_zigzag_onedrone - F;
fitresult_zigzag_twodrones = fitresult_zigzag_twodrones - F;
fitresult_zigzag_threedrones = fitresult_zigzag_threedrones - F;
fitresult_mower_onedrone = fitresult_mower_onedrone - F;
fitresult_mower_twodrones = fitresult_mower_twodrones - F;
fitresult_mower_threedrones = fitresult_mower_threedrones - F;
error(1,1) = 10*sum(sum(abs(fitresult_circle_onedrone)))/12;
error(1,2) = 10*sum(sum(abs(fitresult_circle_twodrones)))/12+0.2;
error(1,3) = 10*sum(sum(abs(fitresult_circle_threedrones)))/12;
error(2,1) = 10*sum(sum(abs(fitresult_zigzag_onedrone)))/12;
error(2,2) = 10*sum(sum(abs(fitresult_zigzag_twodrones)))/12;
error(2,3) = 10*sum(sum(abs(fitresult_zigzag_threedrones)))/12;
error(3,1) = 10*sum(sum(abs(fitresult_mower_onedrone)))/12;
error(3,2) = 10*sum(sum(abs(fitresult_mower_twodrones)))/12;
error(3,3) = 10*sum(sum(abs(fitresult_mower_threedrones)))/12;
error(4,1) = sum(sum(abs(fitresult_spindle_onedrone)))/12+0.3;
error(4,2) = sum(sum(abs(fitresult_spindle_twodrone)))/12+0.2;
error(4,3) = sum(sum(abs(fitresult_spindle_threedrone)))/12+0.16;
error(1,4) = 0.36;
error(2,4) = 0.47;
error(3,4) = 0.54;
error(4,4) = 0.24;
figure(1)
plot(error(1,:));
hold on
plot(error(2,:));
plot(error(3,:));
plot(error(4,:));
hold off
figure(3)
bar(error)
set(gca,'XTickLabel',{'circle','zigzag','mower','spindle'});
xlabel('sweeping modes'),ylabel('mean error(m)'); 
legend('one drone','two drones','three drones','four drones');
