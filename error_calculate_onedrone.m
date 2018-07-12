F =[0 30 15 10;30 -20 5 20;-30 -30 15 30];
Fitresult_1hz =  [    0.0519   30.0966   15.0657    9.9350;
29.9777  -20.2865    4.9281   20.3770;
-30.0242  -30.0323   15.0493   29.9918];

Fitresult_0p1hz =[   -0.8844   22.1442   16.0561    8.2661;
29.8031  -20.5645    4.9688   20.0225;
-29.9718  -30.1231   14.8116   30.2540];

Fitresult_0p2hz =[   -1.8781   29.6859   15.7987    9.7033;
29.7942  -20.4826    5.0589   19.9942;
-29.9917  -30.2586   14.9379   29.9634];

Fitresult_0p4hz =[   -0.3428   29.8557   15.3383    9.8675;
30.1082  -22.0489    4.9690   19.5085;
-30.0538  -29.8636   14.9893   29.9548];

Fitresult_0p6hz =[    0.0125   29.9842   15.0346    9.8682;
30.1339  -20.2430    5.0086   19.7322;
-30.0139  -30.1453   15.0534   30.0010];

Fitresult_0p8hz =[    0.0065   30.0381   15.1056   10.0414;
30.0000  -19.7398    5.0417   19.6498;
-29.9476  -30.1370   14.9712   30.0314];

D_1hz = Fitresult_1hz - F;
D_0p1hz = Fitresult_0p1hz - F;
D_0p2hz = Fitresult_0p2hz - F;
D_0p4hz = Fitresult_0p4hz - F;
D_0p6hz = Fitresult_0p6hz - F;
D_0p8hz = Fitresult_0p8hz - F;
error(1,1) = sum(abs(D_0p1hz(:,1)))/3;
error(1,2) = sum(abs(D_0p2hz(:,1)))/3;
error(1,3) = sum(abs(D_0p4hz(:,1)))/3;
error(1,4) = sum(abs(D_0p6hz(:,1)))/3;
error(1,5) = sum(abs(D_0p8hz(:,1)))/3;
error(1,6) = sum(abs(D_1hz(:,1)))/3;
error(2,1) = sum(abs(D_0p1hz(:,2)))/3;
error(2,2) = sum(abs(D_0p2hz(:,2)))/3;
error(2,3) = sum(abs(D_0p4hz(:,2)))/3;
error(2,4) = sum(abs(D_0p6hz(:,2)))/3;
error(2,5) = sum(abs(D_0p8hz(:,2)))/3;
error(2,6) = sum(abs(D_1hz(:,2)))/3;
error(3,1) = sum(abs(D_0p1hz(:,3)))/3;
error(3,2) = sum(abs(D_0p2hz(:,3)))/3;
error(3,3) = sum(abs(D_0p4hz(:,3)))/3;
error(3,4) = sum(abs(D_0p6hz(:,3)))/3;
error(3,5) = sum(abs(D_0p8hz(:,3)))/3;
error(3,6) = sum(abs(D_1hz(:,3)))/3;
error(4,1) = sum(abs(D_0p1hz(:,4)))/3;
error(4,2) = sum(abs(D_0p2hz(:,4)))/3;
error(4,3) = sum(abs(D_0p4hz(:,4)))/3;
error(4,4) = sum(abs(D_0p6hz(:,4)))/3;
error(4,5) = sum(abs(D_0p8hz(:,4)))/3;
error(4,6) = sum(abs(D_1hz(:,4)))/3;
x = [0.1 0.2 0.4 0.6 0.8 1];
figure(1)
plot(x,error(1,:));
hold on
plot(x,error(2,:));
plot(x,error(3,:));
plot(x,error(4,:));
hold off
figure(3)
bar(error(:,2:6))
set(gca,'XTickLabel',{'mean(x)','mean(y)','sigma(x)','sigma(y)'});
xlabel('distribution parameters'),ylabel('error (dm)');  
legend('0.2 Hz','0.4 Hz','0.6 Hz','0.8 Hz','1.0 Hz');  
Fitresult1 =   [0.0519   30.0966   15.0657    9.9350;
29.9777  -20.2865    4.9281   20.3770;
-30.0242  -30.0323   15.0493   29.9918];

Fitresult2 =[ -0.1048   30.2507   14.9995   10.2495; 
30.3544  -19.9135    5.0432   20.3454; 
-30.1063  -29.9690   15.0210   29.9697];

Fitresult3 = [-0.0076   29.6552   14.9853   10.1829; 
30.1106  -19.6060    5.1996   19.5237;
 -29.7782  -29.9183   14.9989   29.8497];

Fitresult1 = Fitresult1-F;
Fitresult2 = Fitresult2-F;
Fitresult3 = Fitresult3-F;
error2(1,1) = sum(abs(Fitresult1(:,1)))/3;
error2(1,2) = sum(abs(Fitresult2(:,1)))/3;
error2(1,3) = sum(abs(Fitresult3(:,1)))/3;
error2(2,1) = sum(abs(Fitresult1(:,2)))/3;
error2(2,2) = sum(abs(Fitresult2(:,2)))/3;
error2(2,3) = sum(abs(Fitresult3(:,2)))/3;
error2(3,1) = sum(abs(Fitresult1(:,3)))/3;
error2(3,2) = sum(abs(Fitresult2(:,3)))/3;
error2(3,3) = sum(abs(Fitresult3(:,3)))/3;
error2(4,1) = sum(abs(Fitresult1(:,4)))/3;
error2(4,2) = sum(abs(Fitresult2(:,4)))/3;
error2(4,3) = sum(abs(Fitresult3(:,4)))/3;
x = [1 2 3 4];
figure(2)
bar(x,error2);
set(gca,'XTickLabel',{'mean(x)','mean(y)','sigma(x)','sigma(y)'});
xlabel('distribution parameters'),ylabel('error (dm)');  
legend('circle','zigzag','mower');
