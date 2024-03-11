function Ho=Init_OSPA_5_jul(reference,model,H)

% clear;
% clc;
% close all;
% 
% pMD=0;
% pFA=100;
% sigma=0.01;
% 
% H = hom_trans([pi/2,0,pi/4,-1,2,3]);
% Ospa_correspondencia_problem;
dist_ini = inf;
Ho = eye(4);
p=2;
%percentil = 0.25;
tic

%c2 = Init_c(reference,model);
%c2=0.5; %STD
c2=0.03; %4DSTDMDFA
%disp(c1)

for n_ref = 1:size(reference,2)
    for n_mod = 1:size(model,2)
        set_reference = sets_distancias(reference,n_ref);
        set_model = sets_distancias(model,n_mod);
        if n_ref==1 & n_mod==1
            c1 = Init_c(set_reference,set_model);
            disp(c1)
            %disp(c2)
        end
        %
        [dist_fin,H2] = ospa_distE_original(reference,model,set_reference,set_model,c1,c2,p);
        if dist_fin < dist_ini
            dist_ini = dist_fin;
            Ho=H2;
            disp(['fila = ' num2str(n_ref) '| columna = ' num2str(n_mod) ' | Best Cost = ' num2str(dist_ini) ' | c1 = ' num2str(c1) ' | c2 = ' num2str(c2)]);
            disp(table(H,Ho,'VariableNames',{'Hgt','Hest'}))
        end
    end
end
toc

end


function [dist,H] = ospa_distE_original(reference,model,set_reference,set_model,c1,c2,p)

%Calculate sizes of the input point patterns
X=set_reference;
Y=set_model;
n = size(X,2);
m = size(Y,2);

%Calculate cost/weight matrix for pairings - fast method with vectorization
XX= repmat(X,[1 m]);
YY= reshape(repmat(Y,[n 1]),[size(Y,1) n*m]);
D = reshape(double(sqrt(sum((XX-YY).^2))),[n m]);

%disp(c1)
%pause;
%c = 0.05;
D = min(c1,D).^p;
    
Xm = [];
Ym = [];

disp(n)
disp(m)
if n<=m
    Dt = D;
    [ass,~]=assignmentoptimal(Dt);
    for i=1:min(size(Dt))
        if Dt(i,ass(i))<c1^p
            Xm = [Xm,reference(:,i)];
            Ym = [Ym,model(:,ass(i))];
        end
    end
else
    Dt = D';
    [ass,~]=assignmentoptimal(Dt);
    for i=1:min(size(Dt))
        if Dt(i,ass(i))<c1^p
            Xm = [Xm,reference(:,ass(i))];
            Ym = [Ym,model(:,i)];
        end
    end
end

[dist,H] = SVD(reference,model,Xm,Ym,c2,p);


end

function c = Init_c(set_reference,set_model)

%Calculate sizes of the input point patterns
X=set_reference;
Y=set_model;
n = size(X,2);
m = size(Y,2);

%Calculate cost/weight matrix for pairings - fast method with vectorization
XX= repmat(X,[1 m]);
YY= reshape(repmat(Y,[n 1]),[size(Y,1) n*m]);
D = reshape(double(sqrt(sum((XX-YY).^2))),[n m]);
%percentil = 10/min(n,m); %STD
%percentil = 0.25;%1/min(n,m);min(n,m)/n*m
%c = quantile(reshape(D,1,[]),percentil);
%pause;
c = 0.01;
end

function [do,H] = SVD(Xt,Yt,X,Y,c2,p)

CMX = mean(X, 2);
CMY = mean(Y, 2);

Xm = X - repmat(CMX, [1, size(X,2)]);
Ym = Y - repmat(CMY, [1, size(Y,2)]);

W = Xm*Ym';
[U,~,V] = svd(W); % singular value decomposition
R = U*diag([1 1 det(U*V)])*V';

T = CMX - R*CMY; 

Ov = [0,0,0];

H=[R,T;Ov,1];

Yto= model_set_order(Yt,H);

n = size(Xt,2);
m = size(Yto,2);

%Calculate cost/weight matrix for pairings - fast method with vectorization
XX= repmat(Xt,[1 m]);
YY= reshape(repmat(Yto,[n 1]),[size(Yto,1) n*m]);
D = reshape(double(sqrt(sum((XX-YY).^2))),[n m]);


%c = quantile(reshape(D,1,[]),percentil);
%disp(c)
%pause;
D = min(c2,D).^p;


[~,cost]=assignmentoptimal(D);
do = ( 1/max(m,n)*( c2^p*abs(m-n)+ cost )) ^(1/p);

end

function set = sets_distancias(dataset,indice)

n = size(dataset,2);

if indice > n
    msg = 'n no puede ser mayor que el tamaño del set';
    error(msg)
end

Y = dataset(:,indice);
seto = vecnorm(dataset-repmat(Y,[1 n]));
set = [seto;seto];

end

function H = hom_trans(h)

yaw = h(1)*180/pi;
pitch = h(2)*180/pi;
roll = h(3)*180/pi;

R = rotz(yaw)*roty(pitch)*rotx(roll);

Ov = [0 , 0, 0];
tv = [h(4) h(5) h(6)]';

H = [R,tv;Ov,1];

end

function model = model_set_order(model_init,H)

Yh = [model_init ; ones(1,size(model_init,2))];
    
Xh = H*Yh;

model = Xh(1:3,:);

end