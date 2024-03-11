%%%%%%%%%%%%%%%%%%%%%%%%%% Ajustes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
name_dataset = 'bunny';
sigma = 0.01;
pMD = 50;
pFA = 100;
idxH = 1;
index_seed = 1;
caso = "ALL"; 

shuffle = false;
% Opciones a graficar:
%   Model: muestra el problema a resolver y el modelo junto a H(reference).
%   Resultado: Muestra H y H_gt aplicado a referencia, ademas reference e 
%       inv(H) aplicados a model.
%   Error: Muestra H y H_gt aplicado a referencia y los errores RMSE,
%       traslacional y rotacional
%   Paper1: Solamente reference y model, se guarda
%   Paper2: 
graficar = 'Model';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%if crearData
%    create_model_files_new(name_dataset,index_seed,idxH,sigma,pMD,pFA,{caso})
%end

if strcmp(name_dataset,'dragon')
    datos.reference_path = '/Datasets/Dragon/'
    datos.model_path = '/Datasets/Dragon/dragon_model_sigma_0.010_pMD_50.0_pFA_0.txt'
    datos.reference_path_ply =
    datos.model_path_ply = '/Datasets/Dragon/dragon_model_sigma_0.010_pMD_50.0_pFA_0.ply'

    datos.Hgt_path = '/Datasets/Dragon/HG_1.txt'

    datos.Figure_reference_path = '/home/pablo/Desktop/registration_alg/datasets/Original/dragon_vrip_res3.ply';
    datos.Figure_model_path = '/home/pablo/Desktop/registration_alg/datasets/dragon/ALL/H_1/seed_1/dragon_model_sigma_0.000_pMD_70.0_pFA_25_SinDownsampling.txt';
    
elseif strcmp(name_dataset,'bunny')
    datos.reference_path = '/Datasets/Bunny/'
    datos.model_path = '/Datasets/Bunny/dragon_model_sigma_0.010_pMD_50.0_pFA_0.txt'
    datos.reference_path_ply =
    datos.model_path_ply = '/Datasets/Bunny/dragon_model_sigma_0.010_pMD_50.0_pFA_0.ply'

    datos.Hgt_path = '/Datasets/Bunny/HG_1.txt'

    datos.Figure_reference_path = '/home/pablo/Desktop/registration_alg/datasets/Original/bun_zipper.ply';
    datos.Figure_model_path = '/home/pablo/Desktop/registration_alg/datasets/bunny/ALL/H_1/seed_1/bunny_model_sigma_0.000_pMD_70.0_pFA_25_SinDownsampling.txt';
end

datos.reference = normalize_data3d(readpoints(datos.reference_path));
datos.model = readpoints(datos.model_path);

datos.ptReference = pointCloud(datos.reference');
datos.ptModel = pointCloud(datos.model');

datos.Hgt = dlmread(datos.Hgt_path);

Figure_Reference = pcread(datos.Figure_reference_path);
Figure_Reference = Figure_Reference.Location';
datos.Figure_reference = normalize_data3d(Figure_Reference);
datos.Figure_model = readpoints(datos.Figure_model_path);

%.txt correspondiente al set del modelo.
H_gt = datos.Hgt;

model_gt = AplicarH(H_gt,datos.reference);

[Valor_RMSE,H,t] = registrarMetodo(datos,model_gt,metodo,H_gt);

%switch graficar
%    case 'Model'
%        graficarModelo(datos,model_gt,H_gt,H,name_dataset);
%    case 'Resultado'
%       model_est = AplicarH(H,datos.reference);
%       graficarResultado(datos,model_gt,model_est,H,H_gt,name_dataset,index_seed,sigma,pMD,pFA,metodo,texto)
%    case 'Error'
%        model_est = AplicarH(H,datos.reference);
%        [TError,RError] = getError(H_gt,H);
%        texto = sprintf("RMSE Value: %f\nTranslational Error: %f\nRotational Error: %f",Valor_RMSE,TError,RError);
%        graficarError(H,H_gt,name_dataset,index_seed,sigma,pMD,pFA,texto,caso,datos,model_gt,model_est,metodo)
%    case 'Paper1'
%        graficarPaperModelo(name_dataset,datos,H);
%    otherwise
%end

disp("Debug Mode")
tiempo = datestr(datenum(0,0,0,0,0,t),'HH:MM:SS');
%fprintf("Metodo:%s |Trans Error: %d |Rot Error:%d |Tiempo:%s\n",metodo,TError,RError,tiempo)

function [Valor_RMSE,H,t] = registrarMetodo(datos,model_gt,metodo,H_gt)
t1 = tic;
    switch metodo
        case 'PSO'
            %H_Init = Init_OSPA_12_oct(datos.reference,datos.model,datos.Hgt);
            %H_IGCP = IGCP(datos.reference,datos.model,H_Init);
            %H_Init = GoICP_reg(datos.reference_path,datos.model_path,output,0);
            
            %H_Init = datos.Hgt;
            %H = PSO5_MM_reg(datos.reference,datos.model,H_Init,datos);
            %H = PSO5_MM_reg(datos.reference,datos.model,inv(H_Init));
            H_Init = Init_OSPA_5_jul(datos.reference,datos.model,datos.Hgt);
            H_IGCP = IGCP(datos.reference,datos.model,H_Init);
            %H = PSO5_MM_reg_L2(datos.reference,datos.model,eye(4),datos);
            H = PSO5_MM_reg(datos.reference,datos.model,H_IGCP,datos);
            model_est = AplicarH(H,datos.reference);
            Valor_RMSE = RMSE(model_gt,model_est);
        case 'PSOCC'
            H_Init = Init_OSPA_5_jul(datos.reference,datos.model,datos.Hgt);
            H_IGCP = IGCP(datos.reference,datos.model,H_Init);
            H = PSO5_MM_CC_reg(datos.reference,datos.model,inv(H_IGCP));
            model_est = AplicarH(H,datos.reference);
            Valor_RMSE = RMSE(model_gt,model_est);
        otherwise
            disp("Error No se eligio ningun metodo valido")
    end
t = toc(t1);
end

function graficarModelo(datos,model_gt,H_gt,H,name_dataset)
%function graficarModelo(dataset,caso,seed,sigma,pMD,pFA,datos,model_gt)    
    %disp("Graficar Modelo")
    fig = figure(1);
    set(gcf, 'Position', get(0, 'Screensize'));
    %subplot(1,2,1)
    referencept = pointCloud(datos.Figure_reference');
    pointscolor=uint8(zeros(referencept.Count,3));
    pointscolor(:,1)=0;
    pointscolor(:,2)=0;
    pointscolor(:,3)=255;
    referencept.Color=pointscolor;
    modelpt = pointCloud(datos.Figure_model');
    
    %Comparacion Blender Matlab borrar
    %H_gt = [1 0 0 0; 0 0 1 0; 0 -1 0 0; 0 0 0 1];

    model_reg = model_set_order(datos.Figure_model,inv(H_gt));
    modelpt = pointCloud(model_reg');
    
    model_reg_2 = model_set_order(datos.Figure_model,inv(H));
    modelpt_2 = pointCloud(model_reg_2');
    pointscolor=uint8(zeros(modelpt_2.Count,3));
    pointscolor(:,1)=0;
    pointscolor(:,2)=255;
    pointscolor(:,3)=0;
    modelpt_2.Color=pointscolor;
    %Fin Comparacion Blender Matlab borrar
    
    pointscolor=uint8(zeros(modelpt.Count,3));
    pointscolor(:,1)=255;
    pointscolor(:,2)=0;
    pointscolor(:,3)=0;
    modelpt.Color=pointscolor;
    pcshow(referencept)
    set(gca,'FontSize',18)
    hold on
    pcshow(modelpt)
    pcshow(modelpt_2)
    %scatter3(datos.reference(1,:),datos.reference(2,:),datos.reference(3,:),'.')
    %hold on
    %scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'.','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    %scatter3(datos.reference(1,:),datos.reference(2,:),datos.reference(3,:),'o')
    %scatter3(model_gt(1,:),model_gt(2,:),model_gt(3,:),'o','MarkerEdgeColor',[0 1 0])
    %scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    ylabel('Y(meter)')
    xlabel('X(meter)')
    zlabel('Z(meter)')
    %title('Problem to Solve')
    lgd = legend('Reference','Model','Reference','Location','northeast');
    lgdposition = get(lgd,'Position');

    lgd.Position = [lgdposition(1)-0.23,lgdposition(2),lgdposition(3),lgdposition(4)];

    ax = gca;
    ax.XLimMode = 'auto';
    ax.YLimMode = 'auto';
    ax.ZLimMode = 'auto';
    xl = xlim;
    yl = ylim;
    zl = zlim;
    ax.DataAspectRatio = [1 1 1];
%     a = [0 0 0.5];
%     b = [0.5 0 0];
%     c = [0 0.5 0];
% 
%     starts = zeros(3,3);
%     ends = [a;b;c];
% 
%     quiver3(starts(:,1), starts(:,2), starts(:,3), ends(:,1), ends(:,2), ends(:,3))
    %axis equal
    hold off
    
    fig2 = figure(2);
    set(gcf, 'Position', get(0, 'Screensize'));
    ref2model = AplicarH(H_gt,datos.reference);
    %subplot(1,2,2)
    ref2modelpt = pointCloud(ref2model');
    pointscolor=uint8(zeros(ref2modelpt.Count,3));
    pointscolor(:,1)=0;
    pointscolor(:,2)=0;
    pointscolor(:,3)=255;  
    ref2modelpt.Color=pointscolor;
    modelpt = pointCloud(datos.model');
    pointscolor=uint8(zeros(modelpt.Count,3));
    pointscolor(:,1)=255;
    pointscolor(:,2)=0;
    pointscolor(:,3)=0;
    modelpt.Color=pointscolor;
    pcshow(ref2modelpt)
    set(gca,'FontSize',18)
    hold on
    pcshow(modelpt)
    %pcshowpair(ref2modelpt,modelpt,'MarkerSize',0)
    %scatter3(model_gt(1,:),model_gt(2,:),model_gt(3,:),'o')
    %scatter3(ref2model(1,:),ref2model(2,:),ref2model(3,:),'o')
    %hold on
    %scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    %scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    ylabel('Y(meter)')
    xlabel('X(meter)')
    zlabel('Z(meter)')
    xlim([xl(1), xl(2)])
    ylim([yl(1), yl(2)])
    zlim([zl(1), zl(2)])
    %title('Model with and without Noise')
    lgd = legend('Reference','Model','Reference','Location','northeast');
    lgdposition = get(lgd,'Position');

    lgd.Position = [lgdposition(1)-0.23,lgdposition(2),lgdposition(3),lgdposition(4)];
    ax = gca;
    %ax.XLimMode = 'auto';
    %ax.YLimMode = 'auto';
    %ax.ZLimMode = 'auto';
    ax.DataAspectRatio = [1 1 1];
    hold off
    %savePlot('H_gt',name_dataset,fig2)
    
    fig3 = figure(3);
    set(gcf, 'Position', get(0, 'Screensize'));
    ref2model = AplicarH(H,datos.reference);
    %subplot(1,2,2)
    ref2modelpt = pointCloud(ref2model');
    pointscolor=uint8(zeros(ref2modelpt.Count,3));
    pointscolor(:,1)=0;
    pointscolor(:,2)=0;
    pointscolor(:,3)=255;
    ref2modelpt.Color=pointscolor;
    modelpt = pointCloud(datos.model');
    pointscolor=uint8(zeros(modelpt.Count,3));
    pointscolor(:,1)=255;
    pointscolor(:,2)=0;
    pointscolor(:,3)=0;
    modelpt.Color=pointscolor;
    pcshow(ref2modelpt)
    set(gca,'FontSize',18)
    hold on
    pcshow(modelpt)
    %pcshowpair(ref2modelpt,modelpt,'MarkerSize',0)
    %scatter3(model_gt(1,:),model_gt(2,:),model_gt(3,:),'o')
    %scatter3(ref2model(1,:),ref2model(2,:),ref2model(3,:),'o')
    %hold on
    %scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    %scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    ylabel('Y(meter)')
    xlabel('X(meter)')
    zlabel('Z(meter)')
    xlim([xl(1), xl(2)])
    ylim([yl(1), yl(2)])
    zlim([zl(1), zl(2)])
    %title('Model with and without Noise')
    lgd = legend('Reference','Model','Reference','Location','northeast');
    lgdposition = get(lgd,'Position');

    lgd.Position = [lgdposition(1)-0.23,lgdposition(2),lgdposition(3),lgdposition(4)];
    ax = gca;
    %ax.XLimMode = 'auto';
    %ax.YLimMode = 'auto';
    %ax.ZLimMode = 'auto';
    ax.DataAspectRatio = [1 1 1];
    hold off
    %savePlot('H_gt',name_dataset,fig2)
end

function graficarPaperModelo(dataset,datos,H)
    figure
    switch dataset
        case "dragon"
            original_path_ply = './datasets/Original/dragon_vrip.ply';
        case "bunny"
            original_path_ply = './datasets/Original/bun_zipper_res2.ply';
            pose.r = -40;
            pose.p = 0;
            pose.y = 90;
    end

    Hs = zeros(4,4);
    Hs(4,4) = 1;
    R = rotz(pose.r)*roty(pose.p)*rotx(pose.y);
    Hs(1:3,1:3)=R;
    
    
    
    original_cloud = pcread(original_path_ply);
    reference = normalize_data3d(original_cloud.Location');
    reference = AplicarH(Hs,reference);
    %reference_cloud = pointCloud(reference');
    %model = AplicarH(H,reference);
    %model_cloud = pointCloud(model');
    pcshow(reference','r')
    %hold on 
    %pcshow(model','b')
    %hold off
    %grid off
    %set(gca,'visible','off')
end

function graficarResultado(datos,model_gt)
%function graficarModelo(dataset,caso,seed,sigma,pMD,pFA,datos,model_gt)    
    %disp("Graficar Modelo")
    figure
    subplot(1,2,1)
    pcshowpair(datos.ptModel,datos.ptReference)
    hold on
    scatter3(datos.reference(1,:),datos.reference(2,:),datos.reference(3,:),'o')
    scatter3(model_gt(1,:),model_gt(2,:),model_gt(3,:),'o','MarkerEdgeColor',[0 1 0])
    scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    ylabel('Y(meter)')
    xlabel('X(meter)')
    zlabel('Z(meter)')
    title('Problem to Solve')
    legend('Model with Noise','Model without Noise','Reference','Location','Best');
    hold off

    subplot(1,2,2)
    pcshow(datos.ptModel)
    hold on
    scatter3(model_gt(1,:),model_gt(2,:),model_gt(3,:),'o','MarkerEdgeColor',[0 1 0])
    scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+','MarkerEdgeColor',[0.8500 0.3250 0.0980])
    ylabel('Y(meter)')
    xlabel('X(meter)')
    zlabel('Z(meter)')
    title('Model with and without Noise')
    legend('Model with Noise','Model without Noise','Location','Best');
    hold off
end

function graficarError(H,H_gt,dataset,seed,STD,MD,FA,texto,tipo,datos,model_gt,model_est,Metodo)

figure
set(gcf, 'WindowState', 'maximized');

ptModel_gt = pointCloud(model_gt');
ptModel_est = pointCloud(model_est');

pcshowpair(ptModel_est,ptModel_gt)
hold on
scatter3(model_gt(1,:),model_gt(2,:),model_gt(3,:),'o')
scatter3(model_est(1,:),model_est(2,:),model_est(3,:),'+')
%scatter3(est_cloud(1,:),est_cloud(2,:),est_cloud(3,:),'+','green')

%legend('Reference','Model')

ylabel('Y(meter)')
xlabel('X(meter)')
zlabel('Z(meter)')
title(sprintf('%s Solution',Metodo))
%title(sprintf('Example of some Solution'))

a = gca;
a.Position(3) = 0.6;
annotation('textbox', [0.75, 0.5, 0.1, 0.1], 'String', texto)
end

function graficarExperimento2(H,H_gt,dataset,seed,STD,MD,FA,Metodo,texto)

tipo = determinarTipo(STD,MD,FA);
datos = getRefModel(dataset,tipo,seed,STD,MD,FA);

disp(H)
disp(H_gt)
model_gt = AplicarH(H_gt,datos.reference);
model_est = AplicarH(H,datos.reference);
%est_cloud = AplicarH(H,AplicarH(inv(H_gt),datos.model));

plotaux(datos,model_gt,model_est,Metodo,texto)
end

function plotaux(datos,model_gt,model_est,Metodo,texto)
figure
set(gcf, 'WindowState', 'maximized');

% subplot(1,2,1)
% scatter3(datos.reference(1,:),datos.reference(2,:),datos.reference(3,:),'o')
% 
% ylabel('Y(meter)')
% xlabel('X(meter)')
% zlabel('Z(meter)')
% title('Problem to solve')
% hold on
% pcshow(datos.ptReference)
% pcshow(datos.ptModel)
% scatter3(datos.model(1,:),datos.model(2,:),datos.model(3,:),'+')
% hold off
% 
% subplot(1,2,2)
ptModel_gt = pointCloud(model_gt');
ptModel_est = pointCloud(model_est');

pcshowpair(ptModel_est,ptModel_gt)
hold on
scatter3(model_gt(1,:),model_gt(2,:),model_gt(3,:),'o')
scatter3(model_est(1,:),model_est(2,:),model_est(3,:),'+')
%scatter3(est_cloud(1,:),est_cloud(2,:),est_cloud(3,:),'+','green')

%legend('Reference','Model')

ylabel('Y(meter)')
xlabel('X(meter)')
zlabel('Z(meter)')
%title(sprintf('%s Solution',Metodo))
title(sprintf('Example of some Solution'))

a = gca;
a.Position(3) = 0.6;
annotation('textbox', [0.75, 0.5, 0.1, 0.1], 'String', texto)

end

function tipo = determinarTipo(STD,MD,FA)
    if STD ~= 0
        if MD ~= 0
            if FA ~=0
                tipo = "ALL";
            elseif FA == 0
                tipo = "3DSTDMD";
            end
        elseif MD == 0
            if FA ~=0
                tipo = "3DSTDFA";
            elseif FA == 0
                tipo = "2DSTD";
            end
        end
    elseif STD == 0
        if MD ~= 0
            if FA ~=0
                tipo = "3DMDFA";
            elseif FA == 0
                tipo = "2DMD";
            end
        elseif MD == 0
            if FA ~=0
                tipo = "2DFA";
            elseif FA == 0
                tipo = "2DSTD";
            end
        end
    end
end

function norm_value = normalize_data2d( data )
% Normalise values of an array to be between -1 and 1
% original sign of the array values is maintained.                             % Create Data
Amax = max(max(data(1:2,:)));
Amin = min(min(data(1:2,:)));
Range = Amax - Amin;
norm_value = ((data(1:2,:) - Amin)/Range - 0.5) * 2;
norm_value = [norm_value;zeros(1,size(data,2))];
end

function savePlot(identificador,dataset,f)
    DateString = datestr(datetime('now'),'dd-mmmm');
    fig_path = sprintf('./Figures/%s',DateString);
    if ~exist(fig_path, 'dir')
        mkdir(fig_path)
    end

    
    fig_path_fig = sprintf('./Figures/%s/%s_RefModel_%s.fig',DateString,dataset,identificador);
    fig_path_png = sprintf('./Figures/%s/%s_RefModel_%s',DateString,dataset,identificador);
    
    saveas(f,fig_path_fig)
    %saveas(f,fig_path_png)
    saveas(f,fig_path_png,'pdf')

    %set(f,'Units','Inches');
    %pos = get(f,'Position');
    %set(f,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    %print(f,fig_path_png,'-dpdf','-r0')
end

function save_txt(data,filename)

fid = fopen(filename, 'w');

m = size(data,1);
    switch m
        case 3
            n = size(data,2);
            fprintf(fid, '%d\n', n);
                for i=1:n
                   fprintf(fid, '%.6f %.6f %.6f\n', data(1,i), data(2,i), data(3,i)); 
                end
        case 2
            n = size(data,2);
            fprintf(fid, '%d\n', n);
                for i=1:n
                   fprintf(fid, '%.6f %.6f %.6f\n', data(1,i),data(2,i),0); 
                end
        case 1
            n = size(data,2);
            fprintf(fid, '%d\n', n);
                for i=1:n
                   fprintf(fid, '%.6f \n', data(1,i)); 
                end
    end
fclose(fid);
end

function save_H(H,filename)
fileID = fopen(filename, 'w');

for c = 1:4
    fprintf(fileID, '%.15f %.15f %.15f %.15f\n', H(c,1), H(c,2), H(c,3), H(c,4));
end
fclose(fileID);

end