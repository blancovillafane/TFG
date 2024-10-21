function data = extractRPM(helice,krpm)
%% extractRPM: extrae los datos de una matriz "helice" para un cierto régimen de RPM.
%
% helice: matriz de los datos de la hélice de 8 columnas
% krpm: rpm de la hélice en miles
    data = helice((30*krpm-29):30*(krpm),:);
end