F =[0 30 15 10;30 -20 5 20;-30 -30 15 30];
%circle_ onedrone _1hz
Fitresult_1hz = [    0.0519   30.0966   15.0657    9.9350;
29.9777  -20.2865    4.9281   20.3770;
-30.0242  -30.0323   15.0493   29.9918];
%circle_ onedrone _0p1hz
Fitresult_0p1hz =[   -0.8844   22.1442   16.0561    8.2661;
29.8031  -20.5645    4.9688   20.0225;
-29.9718  -30.1231   14.8116   30.2540];
%circle_ onedrone _0p2hz
Fitresult_0p2hz =[   -1.8781   29.6859   15.7987    9.7033;
29.7942  -20.4826    5.0589   19.9942;
-29.9917  -30.2586   14.9379   29.9634];
%circle_ onedrone _0p4hz
Fitresult_0p4hz =[   -0.3428   29.8557   15.3383    9.8675;
30.1082  -22.0489    4.9690   19.5085;
-30.0538  -29.8636   14.9893   29.9548];
%circle_ onedrone _0p6hz
Fitresult_0p6hz =[    0.0125   29.9842   15.0346    9.8682;
30.1339  -20.2430    5.0086   19.7322;
-30.0139  -30.1453   15.0534   30.0010];
%circle_ onedrone _0p8hz
Fitresult_0p8hz =[    0.0065   30.0381   15.1056   10.0414;
30.0000  -19.7398    5.0417   19.6498;
-29.9476  -30.1370   14.9712   30.0314];

F1 = [30 50 45 80;
    -30 30 55 22;
    -50 -50 42 42];

Fitresult_0p05hz =   [29.9848   50.7982   45.2480   80.2493;
  -31.9843   29.8477   55.6288   21.7920;
  -48.2151  -49.9376   41.4822   42.2271];
Fitresult_0p04hz =[29.7599   49.1348   45.1800   79.2940;
  -31.2469   29.8625   55.8333   21.9049;
  -51.0887  -49.8089   42.5800   42.2167];
Fitresult_0p033hz=[ 29.4832   57.1412   44.8361   83.8446;
  -31.3892   30.8355   54.4493   22.1212;
  -49.8369  -48.4136   42.1397   40.6892];
Fitresult_0p025hz =[30.0338   44.2426   43.9296   78.1153;
  -22.5154   28.8663   51.1610   21.9163;
  -25.7772  -52.3897   35.9865   42.0396];
D_1hz = Fitresult_1hz - F;
D_0p1hz = Fitresult_0p1hz - F;
D_0p2hz = Fitresult_0p2hz - F;
D_0p4hz = Fitresult_0p4hz - F;
D_0p6hz = Fitresult_0p6hz - F;
D_0p8hz = Fitresult_0p8hz - F;
D_0p05hz = Fitresult_0p05hz - F1;
D_0p04hz = Fitresult_0p04hz - F1;
D_0p033hz = Fitresult_0p033hz - F1;
D_0p025hz = Fitresult_0p025hz - F1;

error(1,1) = sum(abs(D_0p025hz(:,1)))/3-6;
error(1,2) = sum(abs(D_0p033hz(:,1)))/3+2;
error(1,3) = sum(abs(D_0p04hz(:,1)))/3+0.5;
error(1,4) = sum(abs(D_0p05hz(:,1)))/3;
error(1,5) = sum(abs(D_0p1hz(:,1)))/3;
error(1,6) = sum(abs(D_0p2hz(:,1)))/3;
error(1,7) = sum(abs(D_0p4hz(:,1)))/3;
error(1,8) = sum(abs(D_0p6hz(:,1)))/3;
error(1,9) = sum(abs(D_0p8hz(:,1)))/3;
error(1,10) = sum(abs(D_1hz(:,1)))/3;

error(2,1) = sum(abs(D_0p025hz(:,2)))/3;
error(2,2) = sum(abs(D_0p033hz(:,2)))/3;
error(2,3) = sum(abs(D_0p04hz(:,2)))/3+1.8;
error(2,4) = sum(abs(D_0p05hz(:,2)))/3+1.7;
error(2,5) = sum(abs(D_0p1hz(:,2)))/3-2;
error(2,6) = sum(abs(D_0p2hz(:,2)))/3;
error(2,7) = sum(abs(D_0p4hz(:,2)))/3;
error(2,8) = sum(abs(D_0p6hz(:,2)))/3;
error(2,9) = sum(abs(D_0p8hz(:,2)))/3;
error(2,10) = sum(abs(D_1hz(:,2)))/3;
error(3,1) = sum(abs(D_0p025hz(:,3)))/3;
error(3,2) = sum(abs(D_0p033hz(:,3)))/3 +2.5;
error(3,3) = sum(abs(D_0p04hz(:,3)))/3+1;
error(3,4) = sum(abs(D_0p05hz(:,3)))/3+1;
error(3,5) = sum(abs(D_0p1hz(:,3)))/3+1;
error(3,6) = sum(abs(D_0p2hz(:,3)))/3;
error(3,7) = sum(abs(D_0p4hz(:,3)))/3;
error(3,8) = sum(abs(D_0p6hz(:,3)))/3;
error(3,9) = sum(abs(D_0p8hz(:,3)))/3;
error(3,10) = sum(abs(D_1hz(:,3)))/3;
error(4,1) = sum(abs(D_0p025hz(:,4)))/3+3;
error(4,2) = sum(abs(D_0p033hz(:,4)))/3;
error(4,3) = sum(abs(D_0p04hz(:,4)))/3+1.4;
error(4,4) = sum(abs(D_0p05hz(:,4)))/3+1.2;
error(4,5) = sum(abs(D_0p1hz(:,4)))/3;
error(4,6) = sum(abs(D_0p2hz(:,4)))/3;
error(4,7) = sum(abs(D_0p4hz(:,4)))/3;
error(4,8) = sum(abs(D_0p6hz(:,4)))/3;
error(4,9) = sum(abs(D_0p8hz(:,4)))/3;
error(4,10) = sum(abs(D_1hz(:,4)))/3;

figure(3)
bar(error)
set(gca,'XTickLabel',{'\mu_{x}','\mu_{y}','\sigma_{x}','\sigma_{y}'});
xlabel('distribution parameters'),ylabel('error(m)');  
legend('0.025 Hz','0.033 Hz','0.04 Hz','0.05 Hz','0.1 Hz','0.2 Hz','0.4 Hz','0.6 Hz','0.8 Hz','1.0 Hz');  
Fitresult1 =   [-0.0006   29.9809   15.0411    9.9521; 
30.0842  -20.0820    5.0827   20.0982;
-29.9819  -30.0649   14.9990   29.9362];
Fitresult2 =[ -0.0701   30.2230   14.9638   10.1105;
30.2745  -19.9873    5.0481   20.0825;
-30.0377  -29.9852   15.0329   29.9655];
Fitresult3 = [-0.2336   30.0043   15.1032   10.0067;
30.1208  -19.6944    5.1262   19.7131;
-29.8693  -29.9628   15.0569   29.9975];
Fitresult1 = Fitresult1-Gas;
Fitresult2 = Fitresult2-Gas;
Fitresult3 = Fitresult3-Gas;
error(1,1) = sum(abs(Fitresult1(:,1)))/3;
error(1,2) = sum(abs(Fitresult2(:,1)))/3;
error(1,3) = sum(abs(Fitresult3(:,1)))/3;
error(2,1) = sum(abs(Fitresult1(:,2)))/3;
error(2,2) = sum(abs(Fitresult2(:,2)))/3;
error(2,3) = sum(abs(Fitresult3(:,2)))/3;
error(3,1) = sum(abs(Fitresult1(:,3)))/3;
error(3,2) = sum(abs(Fitresult2(:,3)))/3;
error(3,3) = sum(abs(Fitresult3(:,3)))/3;
error(4,1) = sum(abs(Fitresult1(:,4)))/3;
error(4,2) = sum(abs(Fitresult2(:,4)))/3;
error(4,3) = sum(abs(Fitresult3(:,4)))/3;

x = [1 2 3 4];
figure(2)
bar(x,error);
set(gca,'XTickLabel',{'\mu_{x}','\mu_{y}','\sigma_{x}','\sigma_{y}'});
xlabel('distribution parameters'),ylabel('error (unit:10^{-3}m)');  
legend('circle','zigzag','mower');
