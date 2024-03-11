function datos = getRefModel(dataset,caso,idxH,idxseed,sigma,pMD,pFA)
    %Descripcion:
    %Funcion utilizada para facilitar el manejo de los argumentos de
    %distintos algoritmos, dado un nombre de dataset, caso, seed, sigma,
    %pMD y pFA entrega todos los datos necesarios para cualquier algoritmo
    %
    %Output: una estructura datos con los siguientes campos.
    % reference_path: Path al dataset de referencia en txt
    % model_path: Path al dataset modelo en txt
    % reference_path_ply: Path al dataset de referencia en ply
    % model_path_ply: Path al dataset modelo en ply
    % output_path: Path en el que se deben guardar el tiempo de ejecucion y
    %   el H obtenido
    % reference: Dataset de referencia como matriz 3xn
    % model: Dataset modelo como matriz 3xn
    % ptReference: Dataset de referencia como cloud point
    % ptModel: Dataset modelo como cloud point
    % H_gt: ????????????????????????
    % H_gt_path: ?????????????????????
    
    %trans_path = sprintf('./datasets/%s/roll_%0.3f_pitch_%0.3f_yaw_%0.3f_tx_%0.3f_ty_%0.3f_tz_%0.3f',name_dataset,)
    datos.reference_path = sprintf('datasets/%s/%s_red.txt',dataset,dataset);
    datos.model_path = sprintf('./datasets/%s/%s/H_%d/seed_%d/%s_model_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt',dataset,caso,idxH,idxseed,dataset,sigma,pMD,pFA);

    datos.reference_path_ply = sprintf('./datasets/%s/%s_red.ply',dataset,dataset);
    datos.model_path_ply = sprintf('./datasets/%s/%s/H_%d/seed_%d/%s_model_sigma_%0.3f_pMD_%0.1f_pFA_%i.ply' ...
    ,dataset,caso,idxH,idxseed,dataset,sigma,pMD,pFA); % Path del archivo 

    datos.output_path = sprintf('results/%s//%s/H_%d/seed_%d//H_%s_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt' ...
    ,dataset,caso,idxH,idxseed,dataset,sigma,pMD,pFA);
    
    datos.reference = normalize_data3d(readpoints(datos.reference_path));
    try
        datos.model = readpoints(datos.model_path);
    catch
        fprintf('Fallo en encontrar modelo, con:\n')
        disp(datos.model_path)
    end

    datos.ptReference = pointCloud(datos.reference');
    datos.ptModel = pointCloud(datos.model');

    pcwrite(datos.ptReference,datos.reference_path_ply)
    pcwrite(datos.ptModel,datos.model_path_ply)
    
    datos.Hgt_path = sprintf('datasets/%s/%s/H_%d/HG_%d.txt',dataset,caso,idxH,idxH);
    datos.Hgt = dlmread(datos.Hgt_path);
    
    %Guardar MD_labels solo si es necesario
    switch caso
        case '2DMD'
            MD_labels_path = sprintf('./datasets/%s/%s/H_%d/seed_%d/%s_MD_labels_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt',dataset,caso,idxH,idxseed,dataset,sigma,pMD,pFA);
            datos.referenceCC = readlabels(MD_labels_path);
            datos.existeCC = true;
        case '3DSTDMD'
            MD_labels_path = sprintf('./datasets/%s/%s/H_%d/seed_%d/%s_MD_labels_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt',dataset,caso,idxH,idxseed,dataset,sigma,pMD,pFA);
            datos.referenceCC = readlabels(MD_labels_path);
            datos.existeCC = true;
        case 'ALL'
            MD_labels_path = sprintf('./datasets/%s/%s/H_%d/seed_%d/%s_MD_labels_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt',dataset,caso,idxH,idxseed,dataset,sigma,pMD,pFA);
            datos.referenceCC = readlabels(MD_labels_path);
            datos.existeCC = true;
        otherwise
            datos.existeCC = false;
    end
end