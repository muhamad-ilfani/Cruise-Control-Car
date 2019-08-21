clear
load('Modelsatu.mat')
q=0.5; r=1.8;
%q=0.9; r=1.8; %Paling lumayan lah ya
ni=2; no=2; orde=4;

%Carsim Characteristics
L=3.048; tau=1.5; ds=0.5; sudut_sensor = 10;

%Parameter pengendali MPC
m=2; p=5; Q=q*eye(p*no);
R=r*eye(m*ni); du=zeros(m*ni,1);

umin1=0.15; umin2=1.105; %pembatasan sinyal kendali
umax1=0.265; umax2=1.175; %Aslinya umax1 = 0.265

dumin1=-0.09; dumin2=-0.05; %pembatasan delta u
dumax1=0.09; dumax2=0.05;
%==========================================================================
%Loss Function Initialization
%==========================================================================
temp = zeros(p*no,p*orde);
temp1 = zeros(orde*p,orde);
temp2 = zeros(orde*p,ni);
temp3 = zeros(orde*p,m*ni);
temp4 = zeros(orde*p,1);
delta=[]; upsilon=[]; xi=[];
for j=1:p
    temp((j-1)*no+1:j*no,(j-1)*orde+1:j*orde) = c2;
    temp1((j-1)*orde+1:j*orde,:) = eye(orde);
    delta((j-1)*no+1:j*no,1:ni) = d2;
    xi((j-1)*no+1:j*no,1:no) = ky2;
    for i=1:j
        temp1((j-1)*orde+1:j*orde,1:orde) = temp1((j-1)*orde+1:j*orde,1:orde)*a1;
        temp2((j-1)*orde+1:j*orde,1:ni) = temp2((j-1)*orde+1:j*orde,1:ni) + a1^(i-1)*b1;
        temp4((j-1)*orde+1:j*orde,1) = temp4((j-1)*orde+1:j*orde,1) + a1^(i-1)*kx1;
    end
    temp3((j-1)*orde+1:j*orde,1:ni) = temp2((j-1)*orde+1:j*orde,1:ni);
    for i=1:j+1
        if i<=m
            upsilon((j-1)*no+1:j*no,(i-1)*ni+1:i*ni) = d2;
        end
    end
end
for j=2:m
    for i=j:p
        temp3((i-1)*orde+1:i*orde,(j-1)*ni+1:j*ni) = temp3((i-2)*orde+1:(i-1)*orde,(j-2)*ni+1:(j-1)*ni);
    end
end
psi = temp*temp1;
gamma = temp*temp2;
th = temp*temp3;
phi = temp*temp4;
%==========================================================================
%Constrains
%==========================================================================
dumin = [dumin1;dumin2]; dumax = [dumax1;dumax2];
umin = [umin1;umin2]; umax = [umax1;umax2];
E = [-eye(ni*m,ni*m); eye(ni*m,ni*m)];
F = [-eye(ni*m,ni*m); eye(ni*m,ni*m)];
H = zeros(2*ni*m,ni*m);
e=[];
for j=1:m
    e((j-1)*ni+1:j*ni,:) = -dumin;
    e((j-1)*ni+ni*m+1:j*ni+ni*m,:) = dumax;
    f((j-1)*ni+1:j*ni,:) = -umin;
    f((j-1)*ni+ni*m+1:j*ni+ni*m,:) = umax;
    for i=j:m
        H(:,(j-1)*ni+1:j*ni) = H(:,(j-1)*ni+1:j*ni)+F(:,(i-1)*ni+1:i*ni);
    end
end
A = [E;H];