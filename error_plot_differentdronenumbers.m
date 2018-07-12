error = 100*[0.9318    0.3556    0.2483;
    0.6900    0.4311    0.2908;
    0.3499    0.3552    0.2611];
figure(1)
bar(error)
set(gca,'XTickLabel',{'circle','mower','zigzag'});
xlabel('three sweeping modes'),ylabel('error (unit:10^{-3}m)'); 
legend('one drone','two drones','three drones');