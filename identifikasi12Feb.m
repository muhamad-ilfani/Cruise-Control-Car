%========================================================================
%===                              INISIASI                            === 
%========================================================================
%Deklarasi Variabel
ni=2; no=2; nstate=4;                            %input,output,state
nlrn=0.58; nvalid=1-nlrn;                         %persentase learning, persentase validasi
Ts=0.002; Sim_Time=30; Time=(0:Ts:Sim_Time).';   %waktu sampling, waktu simulasi
N=round(Sim_Time/Ts)+1; Nid=round(nlrn*N);       %jumlah dara, jumlah data identifikasi
Nval=round(nvalid*N);                            %jumlah data untuk validasi
y=[state(:,1) state(:,2)];                       %output dari state
%y=output;
%input=[ut ub];
%Deklarasi Matriks
X1=zeros(N,nstate);X2=zeros(N,nstate);           %matriks state identifikasi tk.1 dan tk.2
e1=zeros(N,no);e2=zeros(N,no);                   %matriks error tk. 1 dan tk.2
y1=zeros(N,no);y2=zeros(N,no);                   %matriks output tk.1 dan tk.2

%========================================================================
%===                       IDENTIFIKASI LEVEL 1                       === 
%========================================================================
%Formula: x(k+1)=a1*state+b1*input+kx1 ; y(k)=c1*state+b1*input+ky1

P1=[state(1:Nid-1,:) input(1:Nid-1,:) ones(Nid-1,1)];   %matriks [x1' u' 1]
th1=(P1.'*P1)\(P1.')*state(2:Nid,:);                    %theta=(inv(PI'*PI))*PI'*psi
a1=th1(1:nstate,:)';
b1=th1(nstate+1:nstate+ni,:)';
kx1=th1(nstate+ni+1,:)';
X1(1,:)=state(1,:);
for k=1:Nid                                             %mencari state baru dari parameter id tk.1
    X1(k+1,:)=(a1*X1(k,:).'+b1*input(k,:).'+kx1).';
end
P11=[X1(1:Nid,:) input(1:Nid,:) ones(Nid,1)];           %mencari pi2
th11=(P11.'*P11)\(P11.')*y(1:Nid,:);                    %theta=(inv(PI'*PI))*PI'*psi
c1=th11(1:nstate,:)';
d1=th11(nstate+1:nstate+ni,:)';
ky1=th11(nstate+ni+1,:)';
x1=state(1,:)';
for k=1:Nid
    y1(k,:)=(c1*x1+d1*input(k,:)'+ky1)';
    x1=a1*x1+b1*input(k,:)'+kx1;
    e1(k,:)=y(k,:)-y1(k,:);
end
x01=x1;

%========================================================================
%===                       IDENTIFIKASI LEVEL 2                       ===
%========================================================================
%Formula: x(k+1)=a2*state+b2*input+kx2 ; y(k)=c2*state+b2*input+ky2*e1
%untuk P2 nilainya sama dengan P1 maka a1=a2 serta b1=b2

P22=[X1(1:Nid,:) input(1:Nid,:) e1(1:Nid,:)];           %mencari pi2
th22=(P22.'*P22)\(P22.')*y(1:Nid,:);                    %theta=(inv(PI'*PI))*PI'*psi
c2=th22(1:nstate,:)';
d2=th22(nstate+1:nstate+ni,:)';
ky2=th22(nstate+ni+1:nstate+ni+no,:)';
x2=state(1,:)';
for k=1:Nid
    y2(k,:)=(c2*x2+d2*input(k,:)'+ky2*e1(k,:)')';
    x2=a1*x2+b1*input(k,:)'+kx1;
    e2(k,:)=y(k,:)-y2(k,:);
end
x02=x2;

%========================================================================
%===                   ANALISA SISTEM IDENTIFIKASI                    ===
%========================================================================

Eigen = eig(a1);                %Menentukan kestabilan sistem
Qc    = rank(ctrb(a1,b1));      %Menentukan Controllability tk.1 dan tk.2
Qo(1) = rank(obsv(a1,c1));      %Menentukan Observability tk.1
Qo(2) = rank(obsv(a1,c2));      %Menentukan Observability tk.2

%==========================================================================
%===                              EVALUASI                              ===
%==========================================================================
for i=1:(Nval)
    y1(Nid+i,:)=(c1*x01+d1*input(Nid+i,:).'+ky1).';
    x01=a1*x01+b1*input(Nid+i,:).'+kx1;
    e1(Nid+i,:)=y(Nid+i,:)-y1(Nid+i,:);
    
    y2(Nid+i,:)=(c2*x02+d2*input(Nid+i,:).'+ky2*e1(Nid+i,:).').';
    x02=a1*x02+b1*input(Nid+i,:).'+kx1;
    e2(Nid+i,:)=y(Nid+i,:)-y2(Nid+i,:);
end

%==========================================================================
%===            Cost Function and Final Output Error (FOE)              ===
%==========================================================================

see1=sum(e1(1:Nid,:).^2); see2=sum(e2(1:Nid,:).^2);
vsee1=sum(e1(Nid+1:N,:).^2); vsee2=sum(e2(Nid+1:N,:).^2);
m1=nstate*nstate+nstate*ni+nstate*1+no*nstate+no*ni+no*1;
m2=nstate*nstate+nstate*ni+nstate*1+no*nstate+no*ni+no*no;
tsee1=sum(e1(1:Nid,:).^2).*((Nid+m1)/(Nid-m1));
tsee2=sum(e2(1:Nid,:).^2).*((Nid+m2)/(Nid-m2));
vtsee1=sum(e1(Nid+1:N,:).^2).*((Nval+m1)/(Nval-m1));
vtsee2=sum(e2(Nid+1:N,:).^2).*((Nval+m2)/(Nval-m2));
Je=[sum(see1);sum(see2)]./Nid;
vJe=[sum(vsee1);sum(vsee2)]./Nval;
FOE=[sum(tsee1);sum(tsee2)]./Nid;
vFOE=[sum(vtsee1);sum(vtsee2)]./Nval;

%==========================================================================
%===                     PLOT GRAFIK IDENTIFIKASI                       ===
%==========================================================================
%----------GRAFIK OUTPUT-------------
figure('Name','Sinyal Kendali Metode 2','NumberTitle','off');
subplot(2,1,1);
plot(Time, input(:,1),'b','LineWidth',1.5);
legend({'ut'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Throttle','FontWeight','bold');
title('Throttle Position','FontWeight','bold');
subplot(2,1,2);
plot(Time, input(:,2),'b','LineWidth',1.5);
legend({'ub'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Pressure (MPa)','FontWeight','bold');
title('Wheel Braking Pressure','FontWeight','bold');

figure('Name','Identifikasi','NumberTitle','off');
subplot(2,1,1);
plot(Time,state(:,1),'k--',Time,y1(:,1),'b',Time,y2(:,1),'r','LineWidth',1.5);
legend({'Output Model ', 'Lv 1', 'Lv 2'});
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Kecepatan [km/j]','FontWeight','bold');
title('Kecepatan Longitudinal','FontWeight','bold');
subplot(2,1,2);
plot(Time,state(:,2),'k--',Time,y1(:,2),'b',Time,y2(:,2),'r','LineWidth',1.5);
legend({'Output Model ', 'Lv 1', 'Lv 2'})
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Jarak [m]','FontWeight','bold');
title('Jarak','FontWeight','bold');

%1.KECEPATAN
%figure('Name','Output1','NumberTitle','off');
%temp_mat=[y(:,1),y1(:,1),y2(:,1)];
%tempmax=max(max(temp_mat));
%tempmin=min(min(temp_mat));
%tempx=ones(size(Time))*Sim_Time*nlrn;
%tempy=linspace(tempmin,tempmax,N).';
%figure
%subplot(1,1,1);
%plot(Time,temp_mat(:,1),'k-',Time,temp_mat(:,2),'b-',Time,temp_mat(:,3),'r--');
%hold on;
%plot(tempx,tempy,'r','LineWidth',1.25);
%grid on;
%xlabel('Time[s]','FontWeight','bold');
%ylabel('{v} [m/s]','FontWeight','bold');
%title('Velocity','FontWeight','bold');
%legend('Out','Lv I','Lv II');

%2.POSISI LONGITUDINAL KENDARAAN
figure('Name','Output2','NumberTitle','off');
temp_mat=[y(:,2),y1(:,2),y2(:,2)];
tempmax=max(max(temp_mat));
tempmin=min(min(temp_mat));
tempx=ones(size(Time))*Sim_Time*nlrn;
tempy=linspace(tempmin,tempmax,N).';
figure
subplot(1,1,1);
plot(Time,temp_mat(:,1),'k-',Time,temp_mat(:,2),'b-',Time,temp_mat(:,3),'r--');
hold on;
plot(tempx,tempy,'r','LineWidth',1.25);
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('{d} [m]','FontWeight','bold');
title('Position','FontWeight','bold');
legend('Out','Lv I','Lv II');

%----------GRAFIK INPUT-------------

figure('Name','INPUT','NumberTitle','off');
subplot(2,1,1);
plot(Time,input(:,1));
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Pulse','FontWeight','bold');
title('Control Signal Throttle','FontWeight','bold');
subplot(2,1,2);
plot(Time,input(:,2));
grid on;
xlabel('Time[s]','FontWeight','bold');
ylabel('Pulse','FontWeight','bold');
title('Control Signal Brake','FontWeight','bold');
