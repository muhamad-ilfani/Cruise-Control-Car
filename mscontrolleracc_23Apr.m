function mscontrolleracc(block)
setup(block);

function setup(block)
% Register the number of ports.
block.NumInputPorts  = 3;
block.NumOutputPorts = 1;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

%Set up block dimension
block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
block.InputPort(1).Dimensions = [20 1];
block.InputPort(2).Dimensions = [2 1];
block.InputPort(3).Dimensions = [4 1];


block.RegBlockMethod('Outputs', @Output);
block.OutputPort(1).Dimensions  = [2 1];

% Register the parameters.
block.NumDialogPrms     = 24;

% Set block sample time
block.SampleTimes = [0.001 0];

block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
block.InputPort(1).SamplingMode = 0.001;
block.InputPort(2).SamplingMode = 0.001;
block.InputPort(3).SamplingMode = 0.001;

block.OutputPort(1).SamplingMode = 0.001;


% Register methods
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',    @InitConditions);
block.RegBlockMethod('Outputs',                 @Output);
block.RegBlockMethod('Update',                  @Update);
%endfunction

function DoPostPropSetup(block)
% Setup Dwork
block.NumDworks = 4;
block.Dwork(1).Name = 'u';
block.Dwork(1).DatatypeID      = 0;
block.Dwork(1).Complexity      = 'Real';
block.Dwork(1).UsedAsDiscState = true;
block.Dwork(1).Dimensions      = 2;
block.Dwork(2).Name = 'ym1';
block.Dwork(2).DatatypeID      = 0;
block.Dwork(2).Complexity      = 'Real';
block.Dwork(2).UsedAsDiscState = true;
block.Dwork(2).Dimensions      = 2;
block.Dwork(3).Name = 'du';
block.Dwork(3).DatatypeID      = 0;
block.Dwork(3).Complexity      = 'Real';
block.Dwork(3).UsedAsDiscState = true;
block.Dwork(3).Dimensions      = 4;
block.Dwork(4).Name = 'x1';
block.Dwork(4).DatatypeID      = 0;
block.Dwork(4).Complexity      = 'Real';
block.Dwork(4).UsedAsDiscState = true;
block.Dwork(4).Dimensions      = 4;
%endfunction

function InitConditions(block)
block.Dwork(1).Data = zeros(2,1);
block.Dwork(2).Data = zeros(2,1);
block.Dwork(3).Data = zeros(4,1);
block.Dwork(4).Data = zeros(4,1);
%endfunction

function Output(block)
block.OutputPort(1).Data = block.Dwork(1).Data;
%endfunction

function Update(block)
%constant parameter
p       = block.DialogPrm(1).Data;
no      = block.DialogPrm(2).Data;
ni      = block.DialogPrm(3).Data;
e       = block.DialogPrm(4).Data;
f       = block.DialogPrm(5).Data;
H       = block.DialogPrm(6).Data;
psi     = block.DialogPrm(7).Data;
gamma   = block.DialogPrm(8).Data;
delta   = block.DialogPrm(9).Data;
phi     = block.DialogPrm(10).Data;
xi      = block.DialogPrm(11).Data;
th      = block.DialogPrm(12).Data;
upsilon = block.DialogPrm(13).Data;
Q       = block.DialogPrm(14).Data;
R       = block.DialogPrm(15).Data;
A       = block.DialogPrm(16).Data;
a1      = block.DialogPrm(17).Data;
b1      = block.DialogPrm(18).Data;
c1      = block.DialogPrm(19).Data;
d1      = block.DialogPrm(20).Data;
kx1     = block.DialogPrm(21).Data;
ky1     = block.DialogPrm(22).Data;
umin    = block.DialogPrm(23).Data;
umax    = block.DialogPrm(24).Data;

%menyusun matrix
Yconst  = block.InputPort(1).Data;
ref     = block.InputPort(2).Data;
x0      = block.InputPort(3).Data;
Ymin    = Yconst(1:10);
Ymax    = Yconst(11:20);
output  = [x0(1);x0(2)];
uprev   = block.Dwork(1).Data;
ym1     = block.Dwork(2).Data;
du      = block.Dwork(3).Data;
x1      = block.Dwork(4).Data;

for i=1:p
    mref((i-1)*no+1:i*no,1)=ref;
end

e1=output-ym1;
epsil  = mref-psi*x0-(gamma+delta)*uprev-phi-xi*e1;
epsil1 = psi*x0+(gamma+delta)*uprev+phi+xi*e1;
B = [e; f-H(:,1:ni)*uprev];

%Output Constraint
A1      = [-(th+upsilon);(th+upsilon)];
A       = [A;A1];
B1      = [Ymin+epsil1;Ymax-epsil1];
B       = [B;B1]

fun =@(xaug) ((th+upsilon)*xaug-epsil)'*Q*((th+upsilon)*xaug-epsil) + xaug'*R*xaug;
options=optimset('Display','off', 'Diagnostics' ,'off','LargeScale','off','Algorithm','active-set','TolFun',1e-3,'TolX',1e-3,'MaxIter',100);
[du,fval,eflag] = fmincon(fun,du,A,B,[],[],[],[],'',options);
u = du(1:ni,:)+uprev;
for i=1:ni
    if u(i) < umin, u(i) = umin(i); end
    if u(i) > umax, u(i) = umax(i); end
end
du1 = du(ni+1:2*ni);
ym1 = c1*a1*x1 + (c1*b1+d1)*uprev + d1*du1 + c1*kx1 + ky1;
x1 = a1*x0+b1*u+kx1;

block.Dwork(1).Data = u;
block.Dwork(2).Data = ym1;
block.Dwork(3).Data = du;
block.Dwork(4).Data = x1;

%figure('Name','INPUT','NumberTitle','off');
%subplot(2,1,1);
%plot(Time,u(:,1));
%grid on;
%xlabel('Time[s]','FontWeight','bold');
%ylabel('Throttle (*100%)','FontWeight','bold');
%title('Throttle','FontWeight','bold');
%subplot(2,2,2);
%plot(Time,u(:,2));
%grid on;
%xlabel('Time[s]','FontWeight','bold');
%ylabel('{Q}_{fr} [Nm]','FontWeight','bold');
%title('Torque FR','FontWeight','bold');
%subplot(2,2,3);
%plot(Time,u(:,3));
%grid on;
%xlabel('Time[s]','FontWeight','bold');
%ylabel('{Q}_{rl} [Nm]','FontWeight','bold');
%title('Torque RL','FontWeight','bold');
%subplot(2,2,4);
%plot(Time,u(:,4));
%grid on;
%xlabel('Time[s]','FontWeight','bold');
%ylabel('{Q}_{rr} [Nm]','FontWeight','bold');
%title('Torque RR','FontWeight','bold');
