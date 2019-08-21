function msrefacc(block);
setup(block);

function setup(block)
% Register the number of ports.
block.NumInputPorts = 1;
block.NumOutputPorts = 3;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

%Set up block dimension
block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
block.InputPort(1).Dimensions = [5 1];

block.RegBlockMethod('Outputs', @Output);
block.OutputPort(1).Dimensions  = 1;    %vx
block.OutputPort(2).Dimensions  = 1;    %d
block.OutputPort(3).Dimensions  = 1;    %d_fol

% Register the parameters.
block.NumDialogPrms     = 4;

% Set block sample time
block.SampleTimes = [0.001 0];

block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
block.InputPort(1).SamplingMode = 0.001;

block.OutputPort(1).SamplingMode = 0.001;
block.OutputPort(2).SamplingMode = 0.001;
block.OutputPort(3).SamplingMode = 0.001;
% Register methods
% Register methods
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions',    @InitConditions);
block.RegBlockMethod('Outputs',                 @Output);
block.RegBlockMethod('Update',                  @Update);
%endfunction

function DoPostPropSetup(block)
% Setup Dwork
block.NumDworks = 6;
block.Dwork(1).Name = 'err';
block.Dwork(1).DatatypeID      = 0;
block.Dwork(1).Complexity      = 'Real';
block.Dwork(1).UsedAsDiscState = true;
block.Dwork(1).Dimensions      = 1;
block.Dwork(2).Name = 'Vx_ref';
block.Dwork(2).DatatypeID      = 0;
block.Dwork(2).Complexity      = 'Real';
block.Dwork(2).UsedAsDiscState = true;
block.Dwork(2).Dimensions      = 1;
block.Dwork(3).Name = 'D_des';
block.Dwork(3).DatatypeID      = 0;
block.Dwork(3).Complexity      = 'Real';
block.Dwork(3).UsedAsDiscState = true;
block.Dwork(3).Dimensions      = 1;
block.Dwork(4).Name = 'D_des_fol';
block.Dwork(4).DatatypeID      = 0;
block.Dwork(4).Complexity      = 'Real';
block.Dwork(4).UsedAsDiscState = true;
block.Dwork(4).Dimensions      = 1;
block.Dwork(5).Name = 'ades_crs';
block.Dwork(5).DatatypeID      = 0;
block.Dwork(5).Complexity      = 'Real';
block.Dwork(5).UsedAsDiscState = true;
block.Dwork(5).Dimensions      = 1;
block.Dwork(6).Name = 'ades_fol';
block.Dwork(6).DatatypeID      = 0;
block.Dwork(6).Complexity      = 'Real';
block.Dwork(6).UsedAsDiscState = true;
block.Dwork(6).Dimensions      = 1;
%endfunction

function InitConditions(block)
block.Dwork(1).Data = zeros(1,1);
block.Dwork(2).Data = zeros(1,1);
block.Dwork(3).Data = zeros(1,1);
block.Dwork(4).Data = zeros(1,1);
block.Dwork(5).Data = zeros(1,1);
block.Dwork(6).Data = zeros(1,1);
%endfunction

function Output(block)
block.OutputPort(1).Data = block.Dwork(2).Data;
block.OutputPort(2).Data = block.Dwork(3).Data;
block.OutputPort(3).Data = block.Dwork(4).Data;
%endfunction

function Update(block)
%constant parameter
Ts             = 0.001;

err_old        = block.Dwork(1).Data;
Vx_des         = block.Dwork(2).Data;
D_des          = block.Dwork(3).Data;
D_des_fol      = block.Dwork(4).Data;
ades_crs_old   = block.Dwork(5).Data;
ades_fol_old   = block.Dwork(6).Data;

tau          = block.DialogPrm(1).Data;%time headway
ds           = block.DialogPrm(2).Data;%ds is the additional distance
L            = block.DialogPrm(3).Data;%panjang kendaraan
sudut_sensor = block.DialogPrm(4).Data;%sudut sensor thd posisi longitudinal


sensor    = block.InputPort(1).Data;
v_x1      = sensor(1);
v_x2      = sensor(2);
distance  = sensor(3);
delta_yaw = sensor(4);
v_rel     = sensor(5);

%desired distance
D_des = L+ds+tau*(v_x2*(1000/3600));
D_des_fol = D_des;

%Switching Proccess
%desired longitudinal velocity

%Parameter PI
%%ki = 0.46;
%kv = 0.7;
%kd = 8;
%d_off = 0.1;

%Parameter PI Untuk Diperlambat
kp = 0.23;
ki = 0.46;
kv = 0.7;
kd = 8;
d_off = 0.1;

%Untuk cek switching
q=1;
w=2;
e=3;
r=4;
y=5;
o=6;

v_set = 70; %Driver velocity set point

err = v_set - v_x2;
ades_crs = kp*err+err_old+ki*err*Ts;
ades_fol = kv*(v_x1-v_x2)+kd*(distance-D_des);

if distance < D_des+0.1 && delta_yaw < sudut_sensor && v_rel > 0
    Vx_des = v_x1;
else 
    Vx_des = v_x2;
end

block.Dwork(1).Data = err;
block.Dwork(2).Data = Vx_des;
block.Dwork(3).Data = D_des;
block.Dwork(4).Data = D_des_fol;
block.Dwork(5).Data = ades_crs;
block.Dwork(6).Data = ades_fol;


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
