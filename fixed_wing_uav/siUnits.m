function matrizSI = siUnits(matriz)
%% matrizSI: TRANSFORMA UNA MATRIZ DE DATOS EN SISTEMA IMPERIAL A SISTEMA INTERNACIONAL
    matrizSI        =   matriz;
    matrizSI(:,1)   =   matriz(:,1).*0.44704;   % Velocidad V
    matrizSI(:,6)   =   matriz(:,6).*745.7;     % Potencia propulsiva P
    matrizSI(:,7)   =   matriz(:,7).*0.11;      % Torque 
    matrizSI(:,8)   =   matriz(:,8).*4.44822;   % Empuje
end