%Metode 2

figure('Name','Kecepatan Metode 2','NumberTitle','off');
plot(Time1, Velocity1(:,1),'b--',Time1,Velocity1(:,2),'r','LineWidth',1.5);
legend({'Vl','Vh'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Velocity [km/h]','FontWeight','bold');
%ylim([45 75]);
%title('Kecepatan Metode Constraint','FontWeight','bold');

figure('Name','Jarak Metode 2','NumberTitle','off');
plot(Time1, dmin,'b--',Time1, dmax, 'm--',Time1,Distance1(:,1),'r','LineWidth',1.5);
legend({'d min ', 'd max', 'd'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Distance [m]','FontWeight','bold');
%%title('Jarak Metode Switching','FontWeight','bold');

figure('Name','Sinyal Kendali Metode 2','NumberTitle','off');
subplot(2,1,1);
plot(Time1, sinyal_kendali1(:,1),'r','LineWidth',1.5);
%legend({'ut'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Throttle [rad]','FontWeight','bold');
%title('Throttle Position','FontWeight','bold');
subplot(2,1,2);
plot(Time1, sinyal_kendali1(:,2),'r','LineWidth',1.5);
%legend({'ub'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Brake Pressure [MPa]','FontWeight','bold');
%title('Wheel Braking Pressure','FontWeight','bold');

%Metode 1

figure('Name','Kecepatan Metode 1','NumberTitle','off');
plot(Time1, Velocity2(:,1),'b--',Time1,Velocity2(:,2),'r','LineWidth',1.5);
legend({'Vl','Vh'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Velocity [km/h]','FontWeight','bold');
%title('Kecepatan Metode Switching','FontWeight','bold');

figure('Name','Jarak Metode 1','NumberTitle','off');
plot(Time1, Distance2(:,2),'b--',Time1,Distance2(:,1),'r','LineWidth',1.5);
legend({'d desire', 'd'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Distance [m]','FontWeight','bold');
%title('Jarak Metode Switching','FontWeight','bold');

figure('Name','Sinyal Kendali Metode 1','NumberTitle','off');
subplot(2,1,1);
plot(Time1, sinyal_kendali2(:,1),'r','LineWidth',1.5);
%legend({'ut'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Throttle [rad]','FontWeight','bold');
%title('Throttle Position','FontWeight','bold');
subplot(2,1,2);
plot(Time1, sinyal_kendali2(:,2),'r','LineWidth',1.5);
%legend({'ub'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Brake Pressure [MPa]','FontWeight','bold');
%title('Wheel Braking Pressure','FontWeight','bold');

%Perbandingan

figure('Name','Perbandingan Kecepatan','NumberTitle','off');
plot(Time1, Velocity2(:,1),'b--',Time1,Velocity2(:,2),'k',Time1,Velocity1(:,2),'r','LineWidth',1.5);
legend({'Vl', 'Vh Switching','Vh Constraint'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Velocity [km/h]','FontWeight','bold');
%title('Kecepatan','FontWeight','bold');

figure('Name','Perbandingan Jarak','NumberTitle','off');
subplot(2,1,1);
plot(Time1, dmin,'b--',Time1, dmax, 'm--',Time1,Distance1(:,1),'r','LineWidth',1.5);
legend({'d min ', 'd max', 'd Constraint'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Distance [m]','FontWeight','bold');
%title('Jarak Metode Switching','FontWeight','bold');
subplot(2,1,2);
plot(Time1, Distance2(:,2),'b--',Time1,Distance2(:,1),'r','LineWidth',1.5);
legend({'d desire', 'd Switching'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Distance [m]','FontWeight','bold');
%title('Jarak Metode Constraint','FontWeight','bold');
