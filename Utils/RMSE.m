function R = RMSE(x,y)
    %Funcion que dados dos vectores calcula RMSE
    R = sqrt(mean((x(:)-y(:)).^2));
end