function msref(block)
setup(block);

function setup(block)
% Register the number of ports.
block.NumInputPorts = 3;
block.NumOutputPorts = 1;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

%Set up block dimension
block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
block.InputPort(1).Dimensions = [4 1]; %Vxl,Xol,Yol,YawRate 
block.InputPort(2).Dimensions = [4 1]; %Vxh,Xoh,Yoh,YawRate
block.InputPort(3).Dimensions = 1; %Distance 

block.RegBlockMethod('Outputs', @Output);
block.OutputPort(1).Dimensions  = [4 1]; %Vxh,d,YawRate,vr

% Register the parameters.
%block.NumDialogPrms     = 7;

% Set block sample time
block.SampleTimes = [0.001 0];

block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
block.InputPort(1).SamplingMode = 0.001;
block.InputPort(2).SamplingMode = 0.001;
block.InputPort(3).SamplingMode = 0.001;

block.OutputPort(1).SamplingMode = 0.001;

% Register methods
block.RegBlockMethod('Outputs',                 @Output);
%endfunction


function Output(block)
data_l  = block.InputPort(1).Data; %leader
data_h  = block.InputPort(2).Data; %host
d       = block.InputPort(3).Data; %distance
vxl     = data_l(1);
xol     = data_l(2);
yol     = data_l(3);
yaw_l   = data_l(4);
vxh     = data_h(1);
xoh     = data_h(2);
yoh     = data_h(3);
yaw_h   = data_h(4);

vr       = vxh - vxl;
yaw_rate = yaw_h - yaw_l;


output   = [vxh;yaw_rate;d;vr]; 
block.OutputPort(1).Data = output;
%endfunction