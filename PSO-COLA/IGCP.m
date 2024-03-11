function H_f=IGCP(reference,model,Ho)

model_r = model_set_order(model,Ho);
dospa_max = ospa_distE_original(reference,model_r,1,2);
%disp(dospa_max)
%dospa = 0;
H_ini = eye(4);

for it=1:400
    
    X=reference;
    Y=model_r;
    n = size(X,2);
    m = size(Y,2);

    XX= repmat(X,[1 m]);
    YY= reshape(repmat(Y,[n 1]),[size(Y,1) n*m]);
    D = reshape(double(sqrt(sum((XX-YY).^2))),[n m]);
    if it==1
        percentil = 1/min(m,n);
        c = quantile(reshape(D,1,[]),percentil);
    end
    p=2;
    D=min(c,D).^p;

    Xm = [];
    Ym = [];

    if n<=m
        Dt = D;
        [ass,~]=assignmentoptimal(Dt);
        for i=1:min(size(Dt))
            if Dt(i,ass(i))<c^p
                Xm = [Xm,X(:,i)];
                Ym = [Ym,Y(:,ass(i))];
            end
        end
    else
        Dt = D';
        [ass,~]=assignmentoptimal(Dt);
        for i=1:min(size(Dt))
            if Dt(i,ass(i))<c^p
                Xm = [Xm,X(:,ass(i))];
                Ym = [Ym,Y(:,i)];
            end
        end
    end
    CMX = mean(Xm, 2);
    CMY = mean(Ym, 2);
    Xm = Xm - repmat(CMX, [1, size(Xm,2)]);
    Ym = Ym - repmat(CMY, [1, size(Ym,2)]);
    W = Xm*Ym';
    [U,~,V] = svd(W); % singular value decomposition
    R = U*diag([1 1 det(U*V)])*V';
    T = CMX - R*CMY; 
    Ov = [0,0,0];
    H=[R,T;Ov,1];
    model_r= model_set_order(Y,H);
    H_f=H*Ho;
    %dospa = ospa_distE_original(reference,model_r,1,2);
    Ho=H_f;

end
disp(H_f)
%disp(dospa)

end

function dist = ospa_distE_original(X,Y,c,p)

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

%Calculate cost/weight matrix for pairings - fast method with vectorization
XX= repmat(X,[1 m]);
YY= reshape(repmat(Y,[n 1]),[size(Y,1) n*m]);
D = reshape(double(sqrt(sum((XX-YY).^2))),[n m]);
D = min(c,D).^p;

[~,cost]=assignmentoptimal(D);

%Calculate final distance    
dist = ( 1/max(m,n)*( c^p*abs(m-n)+ cost )) ^(1/p);

end

function model = model_set_order(model_init,H)

Yh = [model_init ; ones(1,size(model_init,2))];
    
Xh = H*Yh;

model = Xh(1:3,:);

end