function msrefacc(block);
setup(block);

function setup(block)
% Register the number of ports.
block.NumInputPorts = 1;
block.NumOutputPorts = 2;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

%Set up block dimension
block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
block.InputPort(1).Dimensions = [5 1];

block.RegBlockMethod('Outputs', @Output);
block.OutputPort(1).Dimensions  = 20;    %constraint
block.OutputPort(2).Dimensions  = 2;    %ref

% Register the parameters.
block.NumDialogPrms     = 6;

% Set block sample time
block.SampleTimes = [0.001 0];

block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
block.InputPort(1).SamplingMode = 0.001;

block.OutputPort(1).SamplingMode = 0.001;
block.OutputPort(2).SamplingMode = 0.001;
% Register methods
% Register methods
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',    @InitConditions);
block.RegBlockMethod('Outputs',                 @Output);
block.RegBlockMethod('Update',                  @Update);
%endfunction

function DoPostPropSetup(block)
% Setup Dwork
block.NumDworks = 2;
block.Dwork(1).Name = 'Yconst';
block.Dwork(1).DatatypeID      = 0;
block.Dwork(1).Complexity      = 'Real';
block.Dwork(1).UsedAsDiscState = true;
block.Dwork(1).Dimensions      = 20;
block.Dwork(2).Name = 'Des';
block.Dwork(2).DatatypeID      = 0;
block.Dwork(2).Complexity      = 'Real';
block.Dwork(2).UsedAsDiscState = true;
block.Dwork(2).Dimensions      = 2;
%endfunction

function InitConditions(block)
block.Dwork(1).Data = zeros(20,1);
block.Dwork(2).Data = zeros(2,1);
%endfunction

function Output(block)
block.OutputPort(1).Data = block.Dwork(1).Data;
block.OutputPort(2).Data = block.Dwork(2).Data;
%endfunction

function Update(block)
%constant parameter
Yconst       = block.Dwork(1).Data;
Des          = block.Dwork(2).Data;

tau          = block.DialogPrm(1).Data;%time headway
ds           = block.DialogPrm(2).Data;%ds is the additional distance
L            = block.DialogPrm(3).Data;%panjang kendaraan
sudut_sensor = block.DialogPrm(4).Data;%sudut sensor thd posisi longitudinal
ni           = block.DialogPrm(5).Data;
p            = block.DialogPrm(6).Data;

sensor    = block.InputPort(1).Data;
v_x1      = sensor(1);
v_x2      = sensor(2);
distance  = sensor(3);
delta_yaw = sensor(4);
v_rel     = sensor(5);
L_car     = L; %Panjang mobil

%desired distance
D_des = L+ds+tau*(v_x2*(1000/3600));

if distance < D_des+0.1 && delta_yaw < sudut_sensor && v_rel > 0
    Vx_des = v_x1;
else 
    Vx_des = v_x2;
end

Des  = [Vx_des;D_des];

%Output Constraint
Vmin    = v_x1-60; %Bisa 40
Vmax    = v_x1-0.2; %-0.2
dmax    = L_car+ds+20; %0.8 %Percobaan 2 : 20
dmin    = L_car+ds-0.5; %0.5 %tetep
Dmin    = dmin+tau*(v_x2*(1000/3600));
Dmax    = dmax+tau*(v_x2*(1000/3600));
Ymin    = [Vmin;Dmin];
Ymax    = [Vmax;Dmax];
for j=1:p
    Yconst((j-1)*ni+1:j*ni,:) = -Ymin;
    Yconst((j-1)*ni+ni*p+1:j*ni+ni*p,:) = Ymax;
end

block.Dwork(1).Data = Yconst;
block.Dwork(2).Data = Des;


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
