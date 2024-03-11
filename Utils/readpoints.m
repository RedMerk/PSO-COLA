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

