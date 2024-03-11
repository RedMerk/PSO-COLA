function norm_value = normalize_data3d(data,varargin)
% Normalise values of an array to be between -1 and 1
% original sign of the array values is maintained.                             % Create Data
%Amax = max(max(data(1:3,:)))
%Amin = min(min(data(1:3,:)))
%Range = Amax - Amin;
%norm_value = ((data(1:3,:) - Amin)/Range - 0.5) * 2;
%norm_value = [norm_value;zeros(1,size(data,2))];

default_l_bound = 0;
default_u_bound = 1;

p = inputParser;
validScalarNum = @(x) isnumeric(x) && isscalar(x);
validNum = @(x) isnumeric(x);
addRequired(p,'data',validNum);
addOptional(p,'l_bound',default_l_bound,validScalarNum);
addOptional(p,'u_bound',default_u_bound,validScalarNum);
parse(p,data,varargin{:});

data = p.Results.data;
l_bound = p.Results.l_bound;
u_bound = p.Results.u_bound;

xmean = (max(data(1,:))+min(data(1,:)))/2;
ymean = (max(data(2,:))+min(data(2,:)))/2;
zmean = (max(data(3,:))+min(data(3,:)))/2;
data = data-[xmean;ymean;zmean];
    
Amax = max(max(data(1:3,:)));
Amin = min(min(data(1:3,:)));
Range = Amax - Amin;

new_range = u_bound - l_bound;
new_center = (l_bound+u_bound)/2;

s_factor = new_range/Range;

data = data.*s_factor;
norm_value = data+[new_center;new_center;new_center];
end