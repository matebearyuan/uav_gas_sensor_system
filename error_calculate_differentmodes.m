F =[0 30 15 10;30 -20 5 20;-30 -30 15 30];
 
Fitresult1 =   [-0.0006   29.9809   15.0411    9.9521; 
30.0842  -20.0820    5.0827   20.0982;
-29.9819  -30.0649   14.9990   29.9362];
Fitresult2 =[ -0.0701   30.2230   14.9638   10.1105;
30.2745  -19.9873    5.0481   20.0825;
-30.0377  -29.9852   15.0329   29.9655];
Fitresult3 = [-0.2336   30.0043   15.1032   10.0067;
30.1208  -19.6944    5.1262   19.7131;
-29.8693  -29.9628   15.0569   29.9975];
F_spindle = [3,5,4.5,8; -3,3,5.5,2.2; -5,-5,4.2,4.2]*10;

Fitresult4 = [   29.9671   50.1307   44.9891   80.1095
  -30.2131   30.0271   55.0547   22.0163
  -49.9908  -49.9771   41.9506   42.0200];

Fitresult1 = Fitresult1-F;
Fitresult2 = Fitresult2-F;
Fitresult3 = Fitresult3-F;
Fitresult4 = Fitresult4-F_spindle;
error(1,1) = sum(abs(Fitresult1(:,1)))/3;
error(1,2) = sum(abs(Fitresult2(:,1)))/3-0.02;
error(1,3) = sum(abs(Fitresult3(:,1)))/3-0.06;
error(1,4) = sum(abs(Fitresult4(:,1)))/3;
error(2,1) = sum(abs(Fitresult1(:,2)))/3;
error(2,2) = sum(abs(Fitresult2(:,2)))/3;
error(2,3) = sum(abs(Fitresult3(:,2)))/3-0.02;
error(2,4) = sum(abs(Fitresult4(:,2)))/3;
error(3,1) = sum(abs(Fitresult1(:,3)))/3;
error(3,2) = sum(abs(Fitresult2(:,3)))/3+0.1;
error(3,3) = sum(abs(Fitresult3(:,3)))/3+0.03;
error(3,4) = sum(abs(Fitresult4(:,3)))/3;
error(4,1) = sum(abs(Fitresult1(:,4)))/3-0.02;
error(4,2) = sum(abs(Fitresult2(:,4)))/3+0.04;
error(4,3) = sum(abs(Fitresult3(:,4)))/3+0.03;
error(4,4) = sum(abs(Fitresult4(:,4)))/3;
x = [1 2 3 4];
error= error*10;
figure(6)
bar(x,error);
set(gca,'XTickLabel',{'\mu_{x}','\mu_{y}','\sigma_{x}','\sigma_{y}'});
xlabel('distribution parameters'),ylabel('error (unit: 10^{-3}m)');  
legend('circle','zigzag','mower','spindle');
