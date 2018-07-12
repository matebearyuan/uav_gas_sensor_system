f1 = [0,-3,6.5,4];
f2 = [-0.2544+randn*0.8    -2.9399+randn*1.2    6.5928+randn*0.8    4.3378+randn*1.2]; %sweeping
f3 = [0.0575    -3.0187    6.6165    4.6710]; %tracking
f4 = [5.8555    8.0595    4.6077    8.0385];
figure(1)
a = f1(1);b = f1(2);c = f1(3);d = f1(4);
xt = @(t) a+c*cos(t);
yt = @(t) b+d*sin(t);
fplot(xt,yt,'b-','Linewidth',2);
hold on
grid on
plot(a,b,'bo','MarkerSize',6,'Linewidth',1.5);

a = f2(1);b = f2(2);c = f2(3);d = f2(4);
xt = @(t) a+c*cos(t);
yt = @(t) b+d*sin(t);
fplot(xt,yt,'r--','Linewidth',2);
plot(a,b,'rs','MarkerSize',6,'Linewidth',1.5);
a = f3(1);b = f3(2);c = f3(3);d = f3(4);
xt = @(t) a+c*cos(t);
yt = @(t) b+d*sin(t);
fplot(xt,yt,'c-.','Linewidth',2);
plot(a,b,'c^','MarkerSize',6,'Linewidth',1.5);
axis([-5 20 -5 20]);
hold off
error(:,1) = abs(f1-f2)';
error(:,2) = abs(f1-f3)';

x = [1 2 3 4];
error= error*1000;
figure(2)
bar(x,error);
set(gca,'XTickLabel',{'\mu_{x}','\mu_{y}','\sigma_{x}','\sigma_{y}'});
xlabel('distribution parameters'),ylabel('error (unit: 10^{-3}m)');  
legend('sweeping','tracking');