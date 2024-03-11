dataset = 'dragon';

n_Hs = [1];
seeds=[1];
%Sigma = 0.000:0.005:0.1;
Sigma = 0;

%PMD = horzcat(floor(linspace(0,90,10)),[95 96 97 98 99],[25 50 75]);
%PFA= horzcat(floor(linspace(0,200,11)),[25 50 75]);
PMD = 15;
PFA = [25];

remplazarH = false;
casos = {'ALL'};

create_model_files_new(dataset,seeds,n_Hs,Sigma,PMD,PFA,casos)