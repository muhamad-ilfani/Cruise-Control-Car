figure('Name','Sinyal Kendali Metode 2','NumberTitle','off');
subplot(2,1,1);
plot(Time, input(:,1),'b','LineWidth',1.5);
%legend({'ut'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Throttle[rad]','FontWeight','bold');
title('Throttle Position','FontWeight','bold');
subplot(2,1,2);
plot(Time, input(:,2),'b','LineWidth',1.5);
%legend({'ub'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Brake Pressure[MPa]','FontWeight','bold');
title('Wheel Braking Pressure','FontWeight','bold');

figure('Name','Identifikasi','NumberTitle','off');
subplot(2,1,1);
p=plot(Time,state(:,1),'k--',Time,y1(:,1),'b',Time,y2(:,1),'r');
p(1).LineWidth = 2.5;
p(2).LineWidth = 1.5;
p(3).LineWidth = 1.5;
legend({'System Output', 'Level 1', 'Level 2'});
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Velocity[km/h]','FontWeight','bold');
%title('Kecepatan Longitudinal','FontWeight','bold');
subplot(2,1,2);
p=plot(Time,state(:,2),'k--',Time,y1(:,2),'b',Time,y2(:,2),'r','LineWidth',1.5);
p(1).LineWidth = 2.5;
p(2).LineWidth = 1.5;
p(3).LineWidth = 1.5;
legend({'System Output', 'Level 1', 'Level 2'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Distance[m]','FontWeight','bold');
%title('Jarak','FontWeight','bold');