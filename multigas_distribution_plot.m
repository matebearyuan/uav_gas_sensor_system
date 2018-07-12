function fun = single_distribution_plot(fitresult, x, y)
a = 19.98;b=20;c=49.99;d=49.99;
for i = 1:size(x,2)
    for j = 1:size(y,2)
        f1(i,j) = exp(-(x(i)-a)^2/(2*c*c)-(y(j)-b)^2/(2*d*d));
    end
end
a=39.99;b=60.09;c=19.99;d=60.07;
for i = 1:size(x,2)
    for j = 1:size(y,2)
        f2(i,j) = exp(-(x(i)-a)^2/(2*c*c)-(y(j)-b)^2/(2*d*d));
    end
end
figure(4)
surf(x,y,f1');
hold on
axis([min(x),max(x),min(y),max(y)]);
surf(x,y,f2');
hold off