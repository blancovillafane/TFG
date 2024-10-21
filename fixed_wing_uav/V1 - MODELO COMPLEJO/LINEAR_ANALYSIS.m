%% LINEAR ANALYSIS OF THE UAV
% The objective of this script is to analyze the flight modes of a fixed
% wing UAV in a specific operating point given by its linealized model
% matrices A, B, C and D.
% 
% CHANGE THE NAME OF THE STATE SPACE OBJECT (ss) IF NEEDED. By default the
% ss object has the name "linsys1".
% 
% This analysis its necessary in order to develop a Stability Augmentation
% System (SAS) for the Autopilot.
% 
%% Reorganize the state vector and matrices in the correct order
Atemp       = linsys1.A;
A           = zeros(12,12);
Btemp       = linsys1.B;
B           = zeros(12,5);
tempState   = [10 11 12 1 2 3 6 7 8 4 5 6];
% state     = [1 2 3 4 5 6 7 8 9 10 11 12];

for i = 1:12
    for j = 1:12
        A(tempState(i),tempState(j))= Atemp(i,j);
        B(tempState(i),:)           = Btemp(i,:);
    end
end

%% Extract the longitudinal and lateral dynamic matrices
x_long  = [1,3,5,11];
x_lat   = [2,4,6,10];
u_lat   = [1,3];
u_long  = [2,4,5];
[A_long,B_long,C_long,D_long,A_lat,B_lat,C_lat,D_lat] = longAndLatMatrices(A,B,x_long,u_long,x_lat,u_lat);

%% Analyze the different flight modes
longDynamicSys = ss(A_long,B_long,C_long,D_long);
figure, pzplot(longDynamicSys), grid on, title('Longitudinal Dynamics Modes');

latDynamicSys = ss(A_lat,B_lat,C_lat,D_lat);
figure, pzplot(latDynamicSys), grid on, title('Lateral Dynamics Modes');

[Inc_X0_long_modes,E_long,Inc_X0_lat_modes,E_lat] = eigDynamics(A_long,A_lat)
figure, plot(real(E_lat),imag(E_lat),'rx','MarkerSize',1,'LineWidth',20)
hold on, plot(real(E_long),imag(E_long),'bx','MarkerSize',1,'LineWidth',20)

% myzeros = zeros(length(E_lat),1);
% quiver(myzeros,myzeros,real(E_lat),imag(E_lat),'r')
% 
% myzeros = zeros(length(E_long),1);
% quiver(myzeros,myzeros,real(E_long),imag(E_long),'b')

%% AUXILIARY FUNCTIONS USED
function [A_long,B_long,C_long,D_long,A_lat,B_lat,C_lat,D_lat] = longAndLatMatrices(A,B,x_long,u_long,x_lat,u_lat)
% Separates the lateral and longitudinal state matrices, generating 2 
% systems of equations with specific A, B, C and D matrices each.

% --------------------- LATERAL DYNAMICS ----------------------------- %
% x_lat   = [2,4,6,10,12];
num_lat_x = length(x_lat); % number of lateral state var
% u_lat   = [2,3];
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

% --------------------- LONGITUDINAL DYNAMICS -------------------------- %
% x_long  = [1,3,5,9,11];
num_long_x = length(x_long); % number of lateral state var
% u_long  = [1,4];
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
% Computes de eigenvalues and eigenvectors of the state transition
% matrices A_lat and A_long to determine the lateral and longitudinal
% flight modes and the initial conditions to excite those specific modes.

% --------------------- LONGITUDINAL DYNAMICS -------------------------- %
[V_long,D_long] = eig(A_long);        % Tranform. and diagonal matrices

n_long  = length(D_long(:,1));        % number of eigenvalues
E_long  = zeros(n_long,1);
for k = 1:n_long
    E_long(k) = D_long(k,k);          % eigenvalues vector
end                                                                   

long_oscillatory_modes_positions    = oscillatoryModesPositions(E_long);
unique_long_modes_positions         = modesPositions(E_long,long_oscillatory_modes_positions);
E_long=E_long(unique_long_modes_positions);

% Computes the initial conditions to estimulate unique modes
long_variables = [1 3 5 11]; % longitudinal dynamics variables
Inc_X0_long_modes = real(V_long(:,unique_long_modes_positions));

% ------------------------ LATERAL DYNAMICS ---------------------------- %
[V_lat,D_lat] = eig(A_lat);

n_lat   = length(D_lat(:,1));
E_lat  = zeros(n_lat,1);

for k = 1:n_lat
    E_lat(k) = D_lat(k,k);
end

lat_oscillatory_modes_positions     = oscillatoryModesPositions(E_lat);
unique_lat_modes_positions          = modesPositions(E_lat,lat_oscillatory_modes_positions);
E_lat                               = E_lat(unique_lat_modes_positions);

% Computes the initial conditions to estimulate unique modes
lat_variables = [2 4 6 10]; % lateral dynamics variables
Inc_X0_lat_modes = real(V_lat(:,unique_lat_modes_positions));
end

function oscillatory_modes_positions = oscillatoryModesPositions(E)
% DEVUELVE LA POSICION DE LOS MODOS OSCILATORIOS COMPLEJOS UNICOS
% DENTRO DEL VECTOR DE AUTOVALORES. Solo se devuelve la posicion
% del primer número con parte imaginaria positiva.

i       = 1;        % actual mode position number
j       = 1;        % oscillatory_modes_position counter
oscillatory_modes_positions = zeros(length(E),1);
while j<=length(E) && i<=length(E)
    if ~(imag(E(i)) == 0)
        oscillatory_modes_positions(j) = i;
        i = i + 2;                    % +2 to jump over the conjugate pair
    else
        i = i + 1;
    end
    j=j+1;
end
% delete zeros in the vector
oscillatory_modes_positions(oscillatory_modes_positions==0) = [];   
end

function modes_positions = modesPositions(E,oscillatory_modes_positions)
% DEVUELVE LA POSICION DE LOS MODOS UNICOS DENTRO DEL VECTOR DE
% AUTOVALORES. De los pares complejos conjugados solo se devuelve
% la posicion del primer número con parte imaginaria positiva. Los
% autovalores reales se devuelven sus posiones sin modificacion alguna.

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

