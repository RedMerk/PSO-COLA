function [H,BC] = PSO5_MM_reg(reference,model,Ho,datos,output_path)
%PSO con particula Ho

metric.X = reference; %%%Fijo
metric.Y = model; %%% Cambia

params.v= euler_trans(Ho);
parameters_3D;

%% Computing Optimal Translation

metric.CostFunction = @(x) OSPA3(x); % Metric Function;
t = tic;
[H,BC] = PSOfunction3D(metric,params,reference,model,Ho,datos);
time = toc(t);
if exist('output_path', 'var')
    EscribirOutput(H,time,output_path,"PSO")
end

end

function [H,BC] = PSOfunction3D(metric,params,reference,model,Ho,datos)

c=params.c;
p=params.p;

CostFunction = @(x) OSPA3(reference,model,x,c,p);
CostFunction2 = @(x) rmse_error(reference,Ho,x);

nVar = params.nVar;

VarSize = [1 nVar];

%% Parameter of the algorithm

MaxIt = params.MaxIt; %Numero de iteraciones
nPop = params.nPop; %Poblacion

% w = params.w;           % Intertia Coefficient
% wdamp = params.wdamp;   % Damping Ratio of Inertia Coefficient
%wdamp = 0.99;
cmax = 4;         % Personal Acceleration Coefficient
cmin = 0;         % Social Acceleration Coefficient
factor=params.factor;

%% Initialization

% The particle template
empty_particle.Position = [];
empty_particle.Velocity =[];
empty_particle.Cost = [];
empty_particle.Best.Position = [];
empty_particle.Best.Cost = [];

% Create population Array
particle = repmat(empty_particle,[nPop,VarSize]);

%Initialize Global Best
GlobalBest.Cost = inf;

VarMinRoll = params.VarMinRoll;	% Lower Bound of Decision Variables
VarMaxRoll = params.VarMaxRoll;    % Upper Bound of Decision Variables
VarMinPitch = params.VarMinPitch;	% Lower Bound of Decision Variables
VarMaxPitch = params.VarMaxPitch;    % Upper Bound of Decision Variables
VarMinYaw = params.VarMinYaw;	% Lower Bound of Decision Variables
VarMaxYaw = params.VarMaxYaw;    % Upper Bound of Decision Variables
VarMinTx = params.VarMinTx;	% Lower Bound of Decision Variables
VarMaxTx = params.VarMaxTx;    % Upper Bound of Decision Variables
VarMinTy = params.VarMinTy;	% Lower Bound of Decision Variables
VarMaxTy = params.VarMaxTy;    % Upper Bound of Decision Variables
VarMinTz = params.VarMinTz;	% Lower Bound of Decision Variables
VarMaxTz = params.VarMaxTz;    % Upper Bound of Decision Variables

VarMax = [VarMaxRoll,VarMaxPitch,VarMaxYaw,VarMaxTx,VarMaxTy,VarMaxTz];
VarMin = [VarMinRoll,VarMinPitch,VarMinYaw,VarMinTx,VarMinTy,VarMinTz];

MaxVelocityRoll = factor*(VarMaxRoll-VarMinRoll);
MaxVelocityPitch = factor*(VarMaxPitch-VarMinPitch);
MaxVelocityYaw = factor*(VarMaxYaw-VarMinYaw);
MaxVelocityTx = factor*(VarMaxTx-VarMinTx);
MaxVelocityTy = factor*(VarMaxTy-VarMinTy);
MaxVelocityTz = factor*(VarMaxTz-VarMinTz);

VelMax = [MaxVelocityRoll,MaxVelocityPitch,MaxVelocityYaw,MaxVelocityTx,MaxVelocityTy,MaxVelocityTz];
VelMin = -VelMax;

costs = zeros(nPop,1);

rng default;

for i=1:nPop
    %Generate random solution and Ho particle in i=1
    if i==1
        particle(i).H = inv(Ho);
        particle(i).Position=[tform2eul(inv(Ho)),tform2trvec(inv(Ho))];
    else
        particle(i).Position = urand(VarMaxRoll,VarMinRoll,VarMaxPitch,VarMinPitch,VarMaxYaw,VarMinYaw,VarMaxTx,VarMinTx,VarMaxTy,VarMinTy,VarMaxTz,VarMinTz);
        particle(i).H = hom_trans(particle(i).Position);
    end
    
    % Initialize Velocity
    particle(i).Velocity = zeros([1,6]);

    % Evaluation
    particle(i).Cost = CostFunction(particle(i).H);

    % Update the Personal Best
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;

    % Update Global Best
    if particle(i).Best.Cost < GlobalBest.Cost
        GlobalBest = particle(i).Best;
    end

end


BestCosts = zeros(MaxIt,1);
median_cost = zeros(1,MaxIt);
mean_cost = zeros(1,MaxIt);
RMSE_vals = zeros(1,MaxIt);

%% Main Loop of PSO

% f = figure(1);
% set(gcf, 'Position', get(0, 'Screensize'));

%N = size(datos.Figure_model, 2);
%indicesAleatorios = randperm(N, int32(N/32));
%Figure_model = datos.Figure_model(:, indicesAleatorios);
 
%N = size(datos.Figure_reference, 2);
%indicesAleatorios = randperm(N, int32(N/32));
%Figure_reference = datos.Figure_reference(:, indicesAleatorios);

for it=1:MaxIt
    rng(it)
    z = rand(); % initial condition (can be anything from 0 to 1)    
    z = 3*z*(1-z);
    w = 0.5*(rand()+z);
    
    for i=1:nPop

        c1 = cmax + (cmin-cmax)*(it-1)/(MaxIt-1);
        c2 = cmin + (cmax-cmin)*(it-1)/(MaxIt-1);

        % Update Velocity
        
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*rand([1 6]).*(particle(i).Best.Position - particle(i).Position) ...
            + c2*rand([1 6]).*(GlobalBest.Position - particle(i).Position);
        
        % Apply Velocity Limits
        
        particle(i).Velocity = max(particle(i).Velocity, VelMin);
        particle(i).Velocity = min(particle(i).Velocity, VelMax);

        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Apply Lower and Upper Bound Limits
        
        particle(i).Position = max(particle(i).Position, VarMin);
        particle(i).Position = min(particle(i).Position, VarMax);
        particle(i).H = hom_trans(particle(i).Position);
        
        % Evaluation
        particle(i).Cost = CostFunction(particle(i).H);
        costs(i)= particle(i).Cost;

        % Update Personal Best
        if particle(i).Cost < particle(i).Best.Cost

            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;

            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest = particle(i).Best;
            end            

        end

    
     % store the best cost value
     BestCosts(it) = GlobalBest.Cost;
     H = hom_trans(GlobalBest.Position);
     rmse = CostFunction2(H);
 
%      w = w * wdamp;
    
    end
    
    median_cost(it) = median(costs);% quantile(costs,0.5);
    mean_cost(it) = mean(costs);
    RMSE_vals(it) = rmse;
    disp(['Iteration ' num2str(it) ...
    ': Best Cost = ' num2str(BestCosts(it)),': RMSE = ' num2str(rmse)]);

    %%%%%%%%%%%Cambio grafico 18-05-2023%%%%%%%%%%%%%%%%%%%%%%%%
    
    %model_reg = model_set_order(Figure_model,inv(H));
    %model_regCloud = pointCloud(model_reg');
    
    %reference_Cloud = pointCloud(Figure_reference');
    %pointscolor=uint8(zeros(reference_Cloud.Count,3));
    %pointscolor(:,1)=255;
    %pointscolor(:,2)=0;
    %pointscolor(:,3)=0;

    %reference_Cloud.Color = pointscolor;

    %hold on
    %pcshow(reference_Cloud)
    
    %pointscolor=uint8(zeros(model_regCloud.Count,3));
    %pointscolor(:,1)=0;
    %pointscolor(:,2)=0;
    %pointscolor(:,3)=255;

    %model_regCloud.Color = pointscolor;
    
    
%     pcshow(model_regCloud)
%     hold off
    
    % Definir la posición y dirección de la cámara
    %campos([-1 -2 1.75]);  % posición de la cámara
    %camtarget([1 1 0]);     % punto de mira de la cámara
    %campos([-1 0 1.5]);
    %camtarget([1 1 1.25]);
    %if it == 1
    %    camzoom(2.5);
    %end
    
    %ax = gca;
    %ax.XLimMode = 'auto';
    %ax.YLimMode = 'auto';
    %ax.ZLimMode = 'auto';
    %xl = xlim;
    %yl = ylim;
    %zl = zlim;
    %ax.DataAspectRatio = [1 1 1];
    
%     axis off
%     
%     drawnow;
%     %savefig(sprintf('./Figuras_Video/figs%d.pdf',it));
%     
%     set(f,'Units','Inches');
%     %pos = get(f,'Position');
%     %set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
%     %print(f,sprintf('./Figuras_Video_Dragon/figs%d.pdf',it),'-dpdf','-r0')
%     
%     pause(0.01);
%     %grid on
%     cla;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    disp(H);
    BC=BestCosts(it);
    
end

%rng shuffle;

f = figure;
plot(BestCosts,'k')
hold on
plot(median_cost,'b')
plot(mean_cost,'r')
plot(RMSE_vals,'color',[0. .6 .5])
legend('OSPA','Median OSPA','Mean OSPA','RMSE')
set(gca,'FontSize',20)
hold off

end
      
function z = OSPA3(reference,model,H,c,p)

X=reference;
Y=model;
Yt= model_set_order(Y,inv(H));
z = ospa_distE_original(X,Yt,c,p);
  
end

function dist = ospa_distE_original(X,Y,c,p)
%
%B. Vo.  26/08/2007
%Compute Schumacher distance between two finite sets X and Y
%as described in the reference
%[1] D. Schuhmacher, B.-T. Vo, and B.-N. Vo, "A consistent metric for performance evaluation in multi-object 
%filtering," IEEE Trans. Signal Processing, Vol. 56, No. 8 Part 1, pp. 3447� 3457, 2008.
%
%Inputs: X,Y-   matrices of column vectors
%        c  -   cut-off parameter (see [1] for details)
%        p  -   p-parameter for the metric (see [1] for details)
%Output: scalar distance between X and Y
%Note: the Euclidean 2-norm is used as the "base" distance on the region
%

% if nargout ~=1 & nargout ~=3
%    error('Incorrect number of outputs'); 
% end

if isempty(X) && isempty(Y)
    dist = 0; 
    return;
end

if isempty(X) || isempty(Y)
    dist = c;   
    return;
end

%Calculate sizes of the input point patterns
n = size(X,2);
m = size(Y,2);
% 
% %Calculate cost/weight matrix for pairings - fast method with vectorization
XX= repmat(X,[1 m]);
YY= reshape(repmat(Y,[n 1]),[size(Y,1) n*m]);
D = reshape(double(sqrt(sum((XX-YY).^2))),[n m]);
D = min(c,D).^p;

[~,cost]=assignmentoptimal(D);
%cost = sum(diag(D));

%Calculate final distance
dist= ( 1/max(m,n)*( c^p*abs(m-n)+ cost ) ) ^(1/p);

end

function vec = icp_reg(reference,model,Ho)
%Pablo Barrios 28/04/2021
% Last change 29/04/2021
% Old line 9: tform_ICP = pcregistericp(ptModel,ptReference,'MaxIterations',200,'Tolerance',[0.01,0.001],'Extrapolate',true);
% New line 9: tform_ICP = pcregistericp(ptModel,ptReference,'MaxIterations',200,'Tolerance',[0.01,0.001]);
    
model = model_set_order(model,Ho);
ptReference = pointCloud(reference');
ptModel = pointCloud(model');
tform_ICP = pcregistericp(ptModel,ptReference,'MaxIterations',200,'Tolerance',[0.01,0.05]);
H = tform_ICP.T';
vec = euler_trans(H);

end

function vec = euler_trans(H)

R=H(1:3,1:3);
v=rotm2eul(R);
t=H(1:3,4)';
vec=[v,t];

end

function H = hom_trans(h)
% Pablo Barrios  28/04/2021
%
% hom_trans Creates an isometric transformation H, based on rotations and 
% translations 
%
% Inputs: h - Vector which contain rotation and translation informations.
%             The format of it is h=[yaw, pitch, roll, tx, ty, tz].
%
% Output: H - 4x4 Matrix in shape H = [R,t;O(1,3),1];

% Rot_matrix = Rz(yaw)*Ry(pitch)*Rx(roll);

yaw = h(3)*180/pi;
pitch = h(2)*180/pi;
roll = h(1)*180/pi;

R = rotz(yaw)*roty(pitch)*rotx(roll);

Ov = [0 , 0, 0];
tv = [h(4) h(5) h(6)]';

H = [R,tv;Ov,1];

end

function pose = urand(VarMaxRoll,VarMinRoll,VarMaxPitch,VarMinPitch,VarMaxYaw,VarMinYaw,VarMaxTx,VarMinTx,VarMaxTy,VarMinTy,VarMaxTz,VarMinTz)

r = unifrnd(VarMinRoll, VarMaxRoll, 1);
p = unifrnd(VarMinPitch, VarMaxPitch, 1);
y = unifrnd(VarMinYaw, VarMaxYaw, 1);
tx = unifrnd(VarMinTx, VarMaxTx, 1);
ty = unifrnd(VarMinTy, VarMaxTy, 1);
tz = unifrnd(VarMinTz, VarMaxTz, 1);
pose = [r,p,y,tx,ty,tz];

%v = unifrnd(-1,1,[1 6]);
%vf = [VarMaxRoll,VarMaxPitch,VarMaxYaw,VarMaxTx,VarMaxTy,VarMaxTz].*v;


end

function rmse = rmse_error(Reference,Ho,Hest)

RefGT = model_set_order(Reference,Ho);
RefEst = model_set_order(Reference,Hest);

E    = RefGT-RefEst;
SQE  = E.^2;
MSE  = mean(SQE(:));
rmse = sqrt(MSE);

end
