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