%% Metric Parameters

%co=0.5;
%co=0.5;%%Experimento appartment
%co=0.25;
%co=0.1;%%Experimento curvas de nivel!!!
%co=0.5;%%Experimento 3DSTDMD
co = 0.5;

po=2.;
params.c=co;
params.p=po;

%% Early Stopping Parameters
params.n_consecutive_flags = 20;
params.early_stopping_threshold = 0.0001;

%% PSO Parameters

MaxIt= 200;% 1000;
nPop= 50; %50;
%nPop= 60; %50;
%bounds= 1;
nVar = 6;
factor = 0.01;
%w = 1;           % Intertia Coefficient
%wdamp = 0.9;   % Damping Ratio of Inertia Coefficient
cmax = 4;         % Personal Acceleration Coefficient
cmin = 0;
%params.w = w;
%params.wdamp = wdamp;
params.cmax = cmax;
params.cmin = cmin;
params.MaxIt = MaxIt; % Number of Iterations
params.nPop = nPop; % Number of Particles
% dtheta1 = deg2rad(30);
% dtheta2 = deg2rad(30);
% dtheta3 = deg2rad(30);
% dt=0.5;
% dtheta1 = deg2rad(25);%Bueno apartment
% dtheta2 = deg2rad(25);%Bueno apartment
% dtheta3 = deg2rad(90);%Bueno apartment
% dt=1.0;%Bueno apartment
dtheta1 = deg2rad(180);
dtheta2 = deg2rad(90);
dtheta3 = deg2rad(180);
dt=1;
MinRoll = params.v(3)-dtheta1; MaxRoll = params.v(3)+dtheta1;
MinPitch = params.v(2)-dtheta2; MaxPitch = params.v(2)+dtheta2;
MinYaw = params.v(1)-dtheta3; MaxYaw = params.v(1)+dtheta3;
MinTx = params.v(4)-dt; MaxTx = params.v(4)+dt;
MinTy = params.v(5)-dt; MaxTy = params.v(5)+dt;
MinTz = params.v(6)-dt; MaxTz = params.v(6)+dt;
% MinRoll = -dtheta1; MaxRoll = dtheta1;
% MinPitch =-dtheta2; MaxPitch = dtheta2;
% MinYaw =-dtheta3; MaxYaw =  dtheta3;
% MinTx = -dt; MaxTx = dt;
% MinTy = -dt; MaxTy = dt;
%MinTz = -dt*0.1; MaxTz = dt*0.1;%Bueno apartment
%MinTz = -dt; MaxTz = dt;
params.VarMinRoll = MinRoll;
params.VarMaxRoll = MaxRoll;
params.VarMinPitch = MinPitch;
params.VarMaxPitch = MaxPitch;
params.VarMinYaw = MinYaw;
params.VarMaxYaw = MaxYaw;
params.VarMinTx = MinTx;
params.VarMaxTx = MaxTx;
params.VarMinTy = MinTy;
params.VarMaxTy = MaxTy;
params.VarMinTz = MinTz;
params.VarMaxTz = MaxTz;
params.nVar = nVar; % Number of variables
params.factor= factor; % Velocity factor 
