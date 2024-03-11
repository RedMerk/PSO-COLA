function create_model_files_new(pclname,seeds,n_Hs,Sigmas,PMDs,PFAs,casos)
%Pablo Barrios  12/04/2022
% inputs:
% pclname: Nombre del point cloud.
% roll,pitch,yaw: Angulos
% tx,ty,tz: translacion
% Nseed: Numero total de semillas
% Sigma: Maxima cantidad de ruido espacial
% PMD: Maxima cantidad de Miss-Detections
% PFA: Maxima cantidad de False-Alarms.
% output:
% genera 10 archivos por cada caso. ie:
% 10 archivos con std, con sigma = linspace(0,Sigma,10);
% 10 archivos con MD, con sigma = linspace(0,PMD,10);
% 10 archivos con FA, con sigma = linspace(0,PFA,10);

Hs_path = sprintf('datasets/%s/Hs.mat',pclname);
if ~isfile(Hs_path)
    fprintf('No existe Hs en: \n%s\nPor favor cree un archivo Hs.mat con CrearHs.m\n',Hs_path)
else
    Hs = load(Hs_path);
    Hs = Hs.Hs; 
end

Nseed = size(seeds,2);
for idxH = n_Hs
    H = Hs(:,:,idxH);
    fprintf("Progreso:%d/%d\n",idxH,size(n_Hs,2))
    for idxseed = 1:Nseed
        seed = seeds(idxseed);
        for c = 1:size(casos,2)
            caso = casos{c};
            rng(seed,'twister');
            switch caso
                case '2DSTD'
                    sigmas = Sigmas;
                    pMDs = 0;
                    pFAs = 0;
                    escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,H)
                case '2DMD'
                    sigmas = 0;
                    pMDs = PMDs;
                    pFAs = 0;
                    escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,H)
                case '2DFA'
                    sigmas = 0;
                    pMDs = 0;
                    pFAs = PFAs;
                    escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,H)
                case '3DSTDFA'
                    sigmas = Sigmas;
                    pMDs = 0;
                    pFAs = PFAs;
                    escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,H)
                case '3DSTDMD'
                    sigmas = Sigmas;
                    pMDs = PMDs;
                    pFAs = 0;
                    escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,H)
                case 'ALL'
                    sigmas = Sigmas;
                    pMDs = PMDs;
                    pFAs = PFAs;
                    escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,H)
                case 'Teaser'
                    sigmas = Sigmas;
                    pMDs = PMDs;
                    pFAs = 0;
                    escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,H)
            end
        end
    end
end
rng('default')
end

function escribirModelo(pclname,caso,idxH,seed,sigmas,pMDs,pFAs,Ho)
    for sigma = sigmas
        for pMD = pMDs
            for pFA = pFAs
                rng(seed,'twister');
                %Creacion de dataset 
                [Model,MD_Label] = create_model(pclname,sigma,pMD,pFA,Ho,caso);
                
                %Definicion de carpetas
                dataset_path = sprintf('./datasets/%s',pclname);
                case_path = sprintf('%s/%s',dataset_path,caso);
                H_path = sprintf('%s/H_%d',case_path,idxH);
                seed_path = sprintf('%s/seed_%d',H_path,seed);
                if strcmp(caso,'Teaser')
                    model_file = sprintf('%s/%s_model_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt',seed_path,pclname,sigma,pMD,pMD);
                else
                    % Con Downsampling
                    model_file = sprintf('%s/%s_model_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt',seed_path,pclname,sigma,pMD,pFA);
                    % Sin Downsampling
                    %model_file = sprintf('%s/%s_model_sigma_%0.3f_pMD_%0.1f_pFA_%i_SinDownsampling.txt',seed_path,pclname,sigma,pMD,pFA);
                end
                % Con Downsampling
                MD_labels_file = sprintf('%s/%s_MD_labels_sigma_%0.3f_pMD_%0.1f_pFA_%i.txt',seed_path,pclname,sigma,pMD,pFA);
                % Sin Downsampling
                %MD_labels_file = sprintf('%s/%s_MD_labels_sigma_%0.3f_pMD_%0.1f_pFA_%i_SinDownsampling.txt',seed_path,pclname,sigma,pMD,pFA);
                H_file = sprintf('%s/HG_%d.txt',H_path,idxH);
                
                %Creacion de carpetas en caso de no existir
                if ~exist(seed_path, 'dir')
                    if ~exist(H_path, 'dir')
                        if ~exist(case_path, 'dir')
                            if ~exist(dataset_path, 'dir')
                                mkdir(dataset_path);
                            end
                            mkdir(case_path);
                        end
                        mkdir(H_path);
                    end
                    mkdir(seed_path);
                end
                
                %Legado
                %Para crear model con shuffle
%                 ind_mod = randperm(size(Model,2),size(Model,2));
%                 Model = Model(:,ind_mod);
                
                %Guardar dataset, Ho y el MD_labels(de ser necesario)
                save_txt(Model,model_file);
                save_H(Ho,H_file);
                
                %Guardar MD_labels solo si es necesario
                switch caso
                    case '2DMD'
                        save_txt(MD_Label,MD_labels_file);
                    case '3DSTDMD'
                        save_txt(MD_Label,MD_labels_file);
                    case 'ALL'
                        save_txt(MD_Label,MD_labels_file);
                end
            end
        end
    end
end

function [Model_final,MD_labels] = create_model(dataset,sigma,pMD,pFA,H,caso)
% Pablo Barrios  28/04/2021
%
% create_model_data Generate the Model set given a Reference set, based on
% isometric transformation and different source of noise.
%
% Inputs: data - Reference datasets: In this program data can be:
%               {'airplane','bunny','circle','diamond','dragon',
%               'hand','happy','horse'}.
%        sigma - Standard deviation of Gaussian noise with zero mean, in
%                order to distort the reference model.
%        pMD - Percentage of Miss-Detections.
%        pFA - Percentage of False Alarms.
%        pmdseed - Seed on Random Number Generation for reproducibility. 
%        Choose it as integer random number.
%        pfaseed - Seed on Random Number Generation for reproducibility. 
%        Choose it as integer random number.
%        caso - Ayuda a generar el experimento de teaser
%
% Comments: Choose pmdseed and pfaseed to be different.
%
% Output: Model_gMF - Correspond to the Model set created from a Reference
%                     set, after applied a transformation Ho, and adding
%                     spatial noise, sigma, and detection errors MD, and
%                     FA.
%         MD_labels - Corresponds to a vector which contains the at first
%                     element, the number of MD points, and the next elements,
%                     corresponds to the labels missed, with respect to the 
%                     reference set.

%Con Downsampling 
dataset_file = sprintf('./datasets/%s/%s_red.txt',dataset,dataset);
Reference = readpoints(dataset_file);

% Sin Downsampling
% if strcmp(dataset,'dragon')
%     dataset_file = './datasets/Original/dragon_vrip_res3.ply';
% elseif strcmp(dataset,'bunny')
%     dataset_file = './datasets/Original/bun_zipper.ply';
% end
% 
% Reference = pcread(dataset_file);
% Reference = Reference.Location';

H = [-0.115367842515062 0.912136340787993 -0.393316102807029 0.413375856139019
0.267874349750556 -0.352721240514919 -0.896566260370829 0.132359246225410
-0.956521611705212 -0.208794210416636 -0.203645240644993 -0.402459595000590
0.000000000000000 0.000000000000000 0.000000000000000 1.000000000000000];

%H = [-0.0057   -0.9991    0.0430    0.1991
%    0.7183   -0.0340   -0.6949    0.3909
%    0.6957    0.0269    0.7178    0.4593
%         0         0         0    1.0000];

Model_n = normalize_data3d(Reference);
Model_p = model_set_order(Model_n,H);

disp(H)

switch caso
    case '2DSTD'
        Model_final = gauss_noise(Model_p,sigma);
        MD_labels = 1:size(Model_n,2);
    case '2DFA'
        Model_final = FA_model(Model_p,Model_p,pFA);
        MD_labels = 1:size(Model_n,2);
    case '2DMD'
        [Model_final,MD_labels] = MD_model(Model_p,pMD);
    case '3DSTDFA'
        Model_g = gauss_noise(Model_p,sigma);
        Model_final = FA_model(Model_g,Model_g,pFA);
        MD_labels = 1:size(Model_n,2);
    case '3DSTDMD'
        Model_g = gauss_noise(Model_p,sigma);
        [Model_final,MD_labels] = MD_model(Model_g,pMD);
    case 'ALL'
        Model_g = gauss_noise(Model_p,sigma);
        [Model_gM,MD_labels] = MD_model(Model_g,pMD);
        Model_final = FA_model(Model_gM,Model_g,pFA);
    case 'Teaser'
        Model_g = gauss_noise(Model_p,sigma);
        Model_final = Teaser_model(Model_g,pMD);
        MD_labels = 1:size(Model_n,2);
end
        

%Model_p = model_set_order(Model_n,H);
%Model_g = gauss_noise(Model_p,sigma);
%[Model_gM,MD_labels] = MD_model(Model_g,pMD);
%Model_final = FA_model(Model_gM,Model_g,pFA);

end

function modelfa = FA_model(model2,model,pFA)
% Pablo Barrios  28/04/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Last Change occurs in Line 33 and line 36.
% Before 33: sz2 = ceil(sz*pFA/100);
% After 33: sz2 = floor(sz*pFA/100);
% Before 36: %modelfa0 = FAT(:,1:ceil(size(model,2)*pFA/100));
% After 36: Remotion of this line.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FA_model Add points into a Model set. This False Alarms are added at the
% end of a new Model set, which contain previous information above of them.
%
% Inputs: model2 - Initial model set in shape 3-N, with MD and Gaussian noise.              
%         model - model corrupted of with spatial noise.
%         pFA - Percentage of False Alarms.
%         pfaseed - Seed on Random Number Generation for reproducibility. 
%                   Choose it as integer random number.
%
% Output: modelfa - Final model set with False Alarms included at the end 
%                   of the original model set. 
%                   If no MD exist, The dimensions of this is 3-N*(pFA/100). 
%                   If Miss detections exists, then the
%                   dimensions of modelfa are 3-N*(1+(pFA-pMD)/100).
                  
if size(model,2) == 3
    model = model';
    model2 = model2';
end

Maxmodel = [max(model(1,:));max(model(2,:));max(model(3,:))];
Minmodel = [min(model(1,:));min(model(2,:));min(model(3,:))];

med = zeros(3,1);

for i = 1:3
    if abs(Maxmodel(i))>abs(Minmodel(i))
        med(i) = (Maxmodel(i)-Minmodel(i))/2+Minmodel(i);
    else
        med(i) = (Minmodel(i)-Maxmodel(i))/2+Maxmodel(i);
    end
end

Rdeseado = 0.7;

sz = size(model,2);
sz2 = floor(sz*pFA/100);
%FA = [unifrnd(Minmodel(1),Maxmodel(1),1,sz2);unifrnd(Minmodel(2),Maxmodel(2),1,sz2);unifrnd(Minmodel(3),Maxmodel(3),1,sz2)];

FA = zeros(3,sz2);
cont = 1;
while sz2 > cont
    xp = unifrnd(-Rdeseado,Rdeseado);
    yp = unifrnd(-Rdeseado,Rdeseado);
    zp = unifrnd(-Rdeseado,Rdeseado);
    if xp^2 + yp^2 + zp^2 <= Rdeseado^2
        FA(:,cont) = [xp,yp,zp];
        cont = cont+1;
    end
end

%FA = Minmodel + (Maxmodel-Minmodel).*rand(3,sz2);
modelfa = [model2 FA+med];

end

function Model = gauss_noise(Reference,sigma)
% Pablo Barrios  28/04/2021
%
% gauss_noise Add Gaussian Spatial noise to the Reference set with zero mean. 
% 
% Inputs: Reference - Matrix with dimesions 3-N
%         sigma - standard deviation of the Gaussian noise.
%
% Output: Model - Resulting Matrix after adding spatial noise with
%                 dimensions 3-N.

    dim = size(Reference);
    %v = sigma.*randn(dim);
    %disp(max(max(v)));
    %disp(Reference)
    Model = Reference + sigma.*randn(dim);
    
end

function [modelmd, md_labels] = MD_model(model,pMD)
% Pablo Barrios  18/05/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Last Change occurs in Line 25
% Before: set_labels = randperm(sz,ceil((1-pMD/100)*sz));
% After: set_labels = randperm(sz,floor((1-pMD/100)*sz));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MD_model Removes points from a Model set
% 
% Inputs: model - Matrix with dimesions 3-N
%         pMD - Percentage of Miss-Detections.
%         pmdseed - Seed on Random Number Generation for reproducibility. 
%                   Choose it as integer random number.
%
% Output: modelmd - Resulting Matrix after random remotion of N*pMD/100
%                   columns. Then the dimensions of modelmd is
%                   3-N*(1-pMD/100).
%         MD_labels - Corresponds to a vector which contains the at first
%                     element, the number of MD points, and the next elements,
%                     corresponds to the labels missed, with respect to the 
%                     reference set.

sz = size(model,2);
set_labels = randperm(sz,ceil((1-pMD/100)*sz));
set_labels = sort(set_labels);
%md_labels = find(~ismember(1:sz,set_labels));
md_labels = [find(ismember(1:sz,set_labels)),find(~ismember(1:sz,set_labels))];
modelmd = model(:,set_labels);

end

function modelTeaser = Teaser_model(model,pMD)
% Pablo Barrios  28/04/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Teaser_model Reemplaza puntos del modelo por una falsa alarma, respetando
% la correspondencia anterior.
%
% Inputs: model - model corrupted of with spatial noise.
%         pMD - Percentage of Miss detection.
%
% Output: modelTeaser - Modelo final con los puntos reemplazados
%                   Las dimensiones siempre seran iguales que las de model.
                  
if size(model,2) == 3
    model = model';
end

modelTeaser = model;

Maxmodel = [max(model(1,:));max(model(2,:));max(model(3,:))];
Minmodel = [min(model(1,:));min(model(2,:));min(model(3,:))];
sz = size(model,2);
FA = Minmodel + (Maxmodel-Minmodel).*rand(3,sz);

set_labels = randperm(sz,ceil((pMD/100)*sz));
set_labels = sort(set_labels);

%md_labels = find(~ismember(1:sz,set_labels));
modelTeaser(:,set_labels) = FA(:,set_labels);

end

function P = readpoints(filename)
% Pablo Barrios  28/04/2021
%
% readpoints Read a file and extract points.
%
% Inputs: filename - File which contain data information. The format should
%                    be as follows: The first number corresponds to the 
%                    number of points, and after the x, y and z points 
%                    coordinates.
% 
% Output: P - Points in a matrix representation with dimesiones 3-N.

file = fopen(filename, 'r');
N = fscanf(file, '%d', 1);
P = fscanf(file, '%f%f%f', [3,N]);
fclose(file);

end

function save_H(H,filename)
fileID = fopen(filename, 'w');

for c = 1:4
    fprintf(fileID, '%.15f %.15f %.15f %.15f\n', H(c,1), H(c,2), H(c,3), H(c,4));
end
fclose(fileID);

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

yaw = h(1)*180/pi;
pitch = h(2)*180/pi;
roll = h(3)*180/pi;

R = rotz(yaw)*roty(pitch)*rotx(roll);

Ov = [0 , 0, 0];
tv = [h(4) h(5) h(6)]';

H = [R,tv;Ov,1];

end

function model = model_set_order(model_init,H)
% Pablo Barrios  28/04/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Last Changes: Remotion of comments below, from lines 32 to 42.
%
% vmodel=randperm(size(model0,2));
%
% model = model0(:,vmodel);
%
% Yh = model_init;
% 
% Xh = R*Yh;
% 
% vmodel=randperm(size(Xh,2));
% 
% model = Xh(:,vmodel);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% model_set_order Apply an isometric transformation H (Rotation and translation) 
% into the Model set. 
%
% Inputs: model_init - Matrix with dimesions 3-N
%         H - Isometric Transformation in Shape [R,t;O(1x3),1].
%
% Output: model - Transformed Model based on transformation H.

Yh = [model_init ; ones(1,size(model_init,2))];
    
Xh = H*Yh;

model = Xh(1:3,:);

end

function matrixH2file(data,filename)

fid = fopen(filename, 'w');

n = size(data,2);
%fprintf(fid, '%d\n', n);
for i=1:n
   fprintf(fid, '%.6f %.6f %.6f %.6f\n', data(i,1), data(i,2), data(i,3), data(i,4) ); 
end

end