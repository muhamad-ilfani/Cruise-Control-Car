function msrefacc(block)
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
block.OutputPort(1).Dimensions  = 1;    %vx
block.OutputPort(2).Dimensions  = 1;    %d

% Register the parameters.
block.NumDialogPrms     = 4;

% Set block sample time
block.SampleTimes = [0.001 0];

block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
block.InputPort(1).SamplingMode = 0.001;

block.OutputPort(1).SamplingMode = 0.001;
block.OutputPort(2).SamplingMode = 0.001;
% Register methods
block.RegBlockMethod('Outputs',                 @Output);
%endfunction
function InitConditions(block)
ades_crs_old=zeros(1,1);
%endfunction

function Output(block)
tau          = block.DialogPrm(1).Data;%time headway
ds           = block.DialogPrm(2).Data;%ds is the additional distance
L            = block.DialogPrm(3).Data;%panjang kendaraan
sudut_sensor = block.DialogPrm(4).Data;%sudut sensor thd posisi longitudinal


sensor  = block.InputPort(1).Data;
v_x1      = sensor(1);
v_x2      = sensor(2);
distance  = sensor(3);
delta_yaw = sensor(4);
v_rel     = sensor(5);

%desired distance
D_des = L+ds+tau*(v_x2*(1000/3600));

%Switching Proccess
%desired longitudinal velocity

%if distance < D_des+0.1 && delta_yaw < sudut_sensor && v_rel > 0
%    Vx_des = v_x1;
%else 
%    Vx_des = v_x2;
%end
kp = 0.1;
ki = 0.01;
kv = 0.1;
kd = 0.001;
d_off = 0.1;

delta_d = distance - D_des;
ades_crs = kp*(70-v_x2)+ki*(70-v_x2)*0.001;
ades_fol = kv*(v_x1-v_x2)+kd*(distance-D_des);

if delta_d<=0 && v_rel<=0
    Vx_des=v_x1;
elseif delta_d>0 && v_rel<0 && ades_fol<=ades_crs
    Vx_des=v_x1;
elseif delta_d>0 && v_rel<0 && ades_fol>ades_crs
    Vx_des=v_x2;
elseif delta_d<d_off && v_rel>0 && ades_fol<=ades_crs
    Vx_des=v_x1;
elseif delta_d<d_off && v_rel>0 && ades_fol>ades_crs
    Vx_des=v_x2;
elseif delta_d>=d_off && v_rel>=0
    Vx_des=v_x2;
end

block.OutputPort(1).Data = Vx_des;
block.OutputPort(2).Data = D_des;
%endfunction

