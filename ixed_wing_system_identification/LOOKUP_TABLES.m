%clear, close all 
format short
load("data_PER310x6E_arrays.mat")    % Se carga un ejemplo de array tras la importación

PER310x6E=siUnits(PER310x6E);

N = 1:18;   % 1e3 RPM vector
V = 0:1:50; % airspeed vector (m/s)

T = zeros(length(N)+1,length(V));   % Thrust lookup matrix
                                    % +1 row for N=0
M = T;                              % Moment lookup matrix
                                 
for i=1:length(N) % kRPM
    array = extractRPM(PER310x6E,i);
    V_pts = array(:,1);
    M_pts = array(:,7);
    T_pts = array(:,8);
    
    pt      = polyfit(V_pts,T_pts,2);
    T_fun   = @(v) pt(1).*v.^2+pt(2).*v+pt(3);
    
    pm      = polyfit(V_pts,M_pts,2);
    M_fun   = @(v) pm(1).*v.^2+pm(2).*v+pm(3);
    
    T(i+1,:) = T_fun(V); % +1 row bc i=1->T=0
    M(i+1,:) = M_fun(V); % +1 row bc i=1->M=0
end

N = [0,N];  % adds 0 RPM

Nmatrix = zeros(length(V),length(N));
Vmatrix = zeros(length(N),length(V));

for i=1:length(V)
    Nmatrix(i,:) = N;
end

for i=1:length(N)
    Vmatrix(i,:) = V;
end
Vmatrix = Vmatrix';

%   MOTOR MODEL
RPM_function    = @(deltat)9000.*tanh(0.5.*(9.*deltat-4))+9000;  
deltat_pts      = 0.1.*(0:10/6:10);
RPM_pts         = RPM_function(deltat_pts);

propulsiveParameters.Nmatrix    = Nmatrix;
propulsiveParameters.Vmatrix    = Vmatrix;
propulsiveParameters.M          = M';
propulsiveParameters.T          = T';
propulsiveParameters.RPM_pts    = RPM_pts;
propulsiveParameters.deltat_pts = deltat_pts;
save ../propulsiveParameters.mat propulsiveParameters

