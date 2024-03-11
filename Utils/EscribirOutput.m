function EscribirOutput(H,t,output_path,nombre)
paths = split(output_path,"//");
if size(paths,1) == 3
    output_file = paths(3);
    paths = [paths(1) "/" nombre "/" paths(2)];
else
    output_file = paths(2);
    paths = [paths(1) "/" nombre ];
end

output_path = join(paths,'');

if ~exist(output_path, 'dir')
       mkdir(output_path);
end
out = fullfile(output_path,output_file);

fileID = fopen(out,'w');
fprintf(fileID, '%f\n', t);
for c = 1:4
    fprintf(fileID, '%.15f %.15f %.15f %.15f\n', H(c,1), H(c,2), H(c,3), H(c,4));
end
fclose(fileID);  
end