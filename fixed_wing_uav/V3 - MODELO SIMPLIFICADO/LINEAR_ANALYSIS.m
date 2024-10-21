% LINEAR_ANALYSIS. CÁLCULA LOS MODOS DE VUELO Y SUS CONDICIONES INICIALES
% DADAS LAS MATRICES DEL MODELO LINEALIZADO A,B,C Y D

% Separa las dinámicas longitudinal y lateral
B=zeros(12,3);
[A_long,B_long,C_long,D_long,A_lat,B_lat,C_lat,D_lat] = longAndLatMatrices(A_cruise100m,B,[2 4 6 9 11],[1],[1 5 10 12],[2 3]);

% Calcula los modos de vuelo y sus condiciones iniciales
[Inc_X0_long_modes,E_long,Inc_X0_lat_modes,E_lat]  = eigDynamics(A_long,A_lat);


%% ---------------------- FUNCIONES AUXILIARES ---------------------- %%


function [A_long,B_long,C_long,D_long,A_lat,B_lat,C_lat,D_lat] = longAndLatMatrices(A,B,x_long,u_long,x_lat,u_lat)
% Separa las dinámicas longitudinal y lateral alrededor del estado de se equilibrio X0 creando las matrices  y  para cada una de ellas.
% 
% A:  12 x 12 matriz de estado.
% B:  12 x 12 matriz de entradas.
% x:  12 x 1  estado del sistema.
% El vector de estados esta compuesto de los siguientes elementos.
% 
% El vector de entradas del sistema esta compuesto de las siguientes variables.
% 
% 
% Las entradas y  permiten elegir cuales son las variables del estado reducido a utilizar.
% Lateral Dynamics
% State vector 
%     x_lat   = [2,4,6,10,12];      
    num_lat_x = length(x_lat); % number of lateral state var
%     u_lat   = [2,3];   
    num_lat_u = length(u_lat); % number of lateral inputs
    
    A_lat   = zeros(num_lat_x,num_lat_x);
    for i=1:num_lat_x   
        for j=1:num_lat_x
            A_lat(i,j)=A(x_lat(i),x_lat(j));
        end
    end
    
    B_lat=zeros(num_lat_x,num_lat_u);
    for i=1:num_lat_x   
        for j=1:num_lat_u
            B_lat(i,j)=B(x_lat(i),u_lat(j));
        end
    end
    
    C_lat = eye(num_lat_x,num_lat_x);
    
    D_lat = zeros(num_lat_x,num_lat_u);
    
% Longitudinal Dynamics
% State vector  
%     x_long  = [1,3,5,9,11];   
    num_long_x = length(x_long); % number of lateral state var
%     u_long  = [1,4];            
    num_long_u = length(u_long); % number of lateral inputs
    
    A_long=zeros(num_long_x,num_long_x);
    for i=1:num_long_x
        for j=1:num_long_x
            A_long(i,j)=A(x_long(i),x_long(j));
        end
    end
    
    B_long=zeros(num_long_x,num_long_u);
    for i=1:num_long_x   
        for j=1:num_long_u
            B_long(i,j)=B(x_long(i),u_long(j));
        end
    end
    
    C_long = eye(num_long_x,num_long_x);
    
    D_long = zeros(num_long_x,num_long_u);
end


function [Inc_X0_long_modes,E_long,Inc_X0_lat_modes,E_lat] = eigDynamics(A_long,A_lat)
% Computes the number of modes found and returns the initial conditions (state vector evaluated at the initial instant ).
% The dynamic matrices for longitudinal and lateral dynamics (A_long and A_lat) can be diagonalized and then, applying the appropiated transformation (using the matrix ), they can be expressed as 
%  with  and  the eigenvectors of . These eigenvalues and eigenvectors works as parameters that determine the dynamics of the system in the time domain.
% In our case:
% 
% In this way, we only excite the  mode of the aircraft (which is equivalent to only see the "scaling" effect of the  eigenvalue associated with that specific mode, because we are laying in the  eigenvector direction).
% 
% Remember that: 
% 
% Also remember that the initial state vector can be expressed as follows:
%  where   and 
% Longitudinal Dynamics
    [V_long,D_long] = eig(A_long);                  % Tranform. and diagonal matrices
    
    n_long  = length(D_long(:,1));                  % number of eigenvalues
    E_long  = zeros(n_long,1);
    for k = 1:n_long
        E_long(k) = D_long(k,k);                    % eigenvalues vector
    end
% to excite the longitudinal modes of the aircraft
% It computes the amount of increment in the initial state vector such that the initial state vector corresponds with the real part of an eigenvector  of matrix , therefore the  mode of the aircraft will be excited alone. The equation of the increment is:
% 
% It can be the real or the imaginary part of any of the complex eigenvector associated with and specific mode.
    %% Computes the unique eigenvalues
%     n_long_oscillatory_modes = sum(~(imag(E_long)==0))/2;            % dividing by 2 cause the complex 
%                                                                      % eigenvalues come in pairs

    long_oscillatory_modes_positions    = oscillatoryModesPositions(E_long);
    unique_long_modes_positions         = modesPositions(E_long,long_oscillatory_modes_positions);
%     long_oscillatory_eigenvalues        = E_long(long_oscillatory_modes_positions);

    E_long=E_long(unique_long_modes_positions);
    
    %% Computes the initial conditions to estimulate unique modes
    % longitudinal dynamics variables
    long_variables = [2 4 6 9 11];
   
    % compute the initial coditions to excite longitudinal modes
     Inc_X0_long_modes = real(V_long(:,unique_long_modes_positions));

    % Computes the same but for a StateSpace of 12x12
    %Inc_X0_long_modes_NONLINEAR = Inc_X0_generator(V_long,long_variables,unique_long_modes_positions);
%%  Lateral Dynamics
    [V_lat,D_lat] = eig(A_lat);
    
    n_lat   = length(D_lat(:,1));
    E_lat  = zeros(n_lat,1);
    
    for k = 1:n_lat
        E_lat(k) = D_lat(k,k);
    end
% to excite the longitudinal modes of the aircraft
% It computes the amount of increment in the initial state vector such that the initial state vector corresponds with the real part of an eigenvector  of matrix , therefore the  mode of the aircraft will be excited alone. The equation of the increment is:
% 
% It can be the real or the imaginary part of any of the complex eigenvector associated with and specific mode.
     %% Computes the unique eigenvalues
%     n_lat_oscillatory_modes = sum(~(imag(E_lat)==0))/2;         % dividing by 2 cause the complex 
%                                                                 % eigenvalues come in pairs
    lat_oscillatory_modes_positions     = oscillatoryModesPositions(E_lat);
    unique_lat_modes_positions          = modesPositions(E_lat,lat_oscillatory_modes_positions);
    E_lat                               = E_lat(unique_lat_modes_positions);
%     lat_oscillatory_eigenvalues = E_lat(lat_oscillatory_modes_positions);
    
    %% Computes the initial conditions to estimulate unique modes
    % lateral dynamics variables
    lat_variables = [1 5 10 12];
    
    % Compute the initial coditions to excite lateral modes
    Inc_X0_lat_modes = real(V_lat(:,unique_lat_modes_positions));
   
    % Computes the same but for a StateSpace of 12x12. Not our case
    %Inc_X0_lat_modes_NONLINEAR = Inc_X0_generator(V_lat,lat_variables,unique_lat_modes_positions);

    function oscillatory_modes_positions = oscillatoryModesPositions(E)
        % DEVUELVE LA POSICION DE LOS MODOS OSCILATORIOS COMPLEJOS UNICOS DENTRO DEL VECTOR DE
        % AUTOVALORES. Solo se devuelve la posicion del primer número con parte imaginaria 
        % positiva.

        i       = 1;        % actual mode position number
        j       = 1;        % oscillatory_modes_position counter
        oscillatory_modes_positions = zeros(length(E),1);
        while j<=length(E) && i<=length(E)
            if ~(imag(E(i)) == 0)
                oscillatory_modes_positions(j) = i;
                i = i + 2;                         % +2 to jump over the conjugate pair
            else
                i = i + 1;
            end
            j=j+1;
        end
        oscillatory_modes_positions(oscillatory_modes_positions==0) = [];   % clear the vector of zeros
    end

    function modes_positions = modesPositions(E,oscillatory_modes_positions)
        % DEVUELVE LA POSICION DE LOS MODOS UNICOS DENTRO DEL VECTOR DE
        % AUTOVALORES. De los pares complejos conjugados solo se devuelve
        % la posicion del primer número con parte imaginaria positiva.

        i       = 1;        % actual mode position number
        j       = 1;        % [modes_position] counter
        modes_positions = zeros(length(E),1);
        while j<=length(E) && i<=length(E)
            if (sum(i==oscillatory_modes_positions))
                modes_positions(j) = i;
                i = i + 2;                         % +2 to jump over the conjugate pair
            else
                modes_positions(j) = i;
                i = i + 1;
            end
            j=j+1;
        end
        modes_positions(modes_positions==0) = [];   % clear the vector of zeros
    end

    
    function Inc_X0_modes = Inc_X0_generator(V,variables,modes_positions)
        % CREA EL VECTOR DE INCREMENTO PARA STATE SPACE 12x12
        
        % we chose the real part of the eigenvectors of the modes
        X0_modes_compact = real([V(:,modes_positions)]); 
    
        % preallocate zeros in the initial state mode matrix with 12 rows
        % for variables ~= input "[variables]"  --> Inc_X0 = 0,  no increment
        Inc_X0_modes = zeros(12,length(modes_positions));

        for i=1:length(modes_positions)
            % just for variables = input "[variables]" 
            % --> Inc_X0 = X0modes
            
            Inc_X0_modes(variables,i)=X0_modes_compact(:,i);
        end                                               
    end
end