function [modeloH] = AplicarH(H,model)
    %Funcion que dado una matriz H 4x4 y un modelo como matriz, retorna el
    %modelo ground truth
    dim = size(model);
    modeloH = H*[model;repmat([1],1,dim(2))];
    modeloH(4,:) = [];
end