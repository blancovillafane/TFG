%% CONFIGURATION SCRIPT - VALIDATION V2
% 
%  This script sets the default the parameters of the Simulink Simulators. 
%  These parameters should be ajusted in order to simulate the
%  identified UAV.
% 
%  All variables most be expressed in SI UNITS.
% 
%% Load Propulsion Data
run('LOOKUP_TABLES.m');
load ../fittedCoefficients.mat;
ST = 0.01;              % SAMPLE TIME
%% Geometry and mass properties

% =============================== %

b           = 2.9;      % wing span  
S           = 0.55;     % wing surface
c_bar       = 0.19;     % mean chord  
delta_max   = 0.35;     % max deflection of the control surfaces 

% =============================== %

mass    = 11;           % Mass of the AC
Jx      = 0.8244;
Jy      = 1.135;
Jz      = 1.759;
Jxz     = 0.120;
J       = [Jx,    0,      -Jxz;...
           0,     Jy,        0;...
           -Jxz,   0,      Jz];     % Inertia Tensor
Jxy     = 0;
% =============================== %

% Computation of gamma inertial variables
gamma   = Jx*Jz-Jxz^2;
gamma1  = Jxz*(Jx - Jy + Jz)/(gamma);
gamma2  = (Jz*(Jz - Jy)+Jxz^2)/gamma;
gamma3  = Jz/gamma;
gamma4  = Jxz/gamma;
gamma5  = (Jz-Jx)/Jy;
gamma6  = Jxz/Jy;
gamma7  = (Jx*(Jx - Jy) + Jxz^2)/gamma;
gamma8  = Jx/gamma; 

% =============================== %
%% Simulation initial conditions and atmospheric conditions

latitude0   = 37.431430;            
longitude0  = -5.859596;
LatLong0    = [latitude0,longitude0];

% =============================== %

% Controlled Wind Gust
gust_start_time = 20;           % (s)
gust_length     = [120 120 80]; % (m)
gust_amplitude  = [3.5 3.5 3.0];% (m/s) Body Axis
 
%  Click the check-boxes in the Simulink Block 
%  "UAV/Enviroment Model/Wind Models/Discrete Wind Gust Model" 
%  to activated the gust.

% =============================== %

% Controlled Uniform Wind around space
Vw_unif         = [0 0 0]';     % (m/s) NED Axis

% =============================== %
%% Propulsion 
maxRPM = 11e3;

% V3 PROPULSION
C_prop  = 1;
k_motor = 80;
 
% V1 PROPULSION
Sprop   = 0.2027;
i0      = 1.5;      % (A)
Vmax    = 44.4;     % (V)
R       = 0.042;    % (ohm)
Kv      = 0.0659;   % (V*s/rad)
KQ      = 0.0659;   % (N*m)
%% Aerodynamics

% forces
CDp         = xfit(1);
k2          = xfit(2);
k1          = xfit(3);
CDq         = xfit(4);
CD_deltae   = xfit(5);

CL0         = xfit(6);
CL_alpha    = xfit(7);
CLq         = xfit(8);
CL_deltae   = xfit(9);

Cy_beta = xfit(10);
Cy_p    = xfit(11);
Cy_r    = xfit(12);
Cy_da   = xfit(13);
Cy_dr   = xfit(14);

% moments
Cl_beta = xfit(15);   
Cl_p    = xfit(16);  
Cl_r    = xfit(17);  
Cl_dr   = xfit(18); 
Cl_da   = xfit(19);

Cm0         = xfit(20);
Cm_alpha    = xfit(21);
Cm_q        = xfit(22);
Cm_de       = xfit(23);

Cn_beta = xfit(24);
Cn_p    = xfit(25);
Cn_r    = xfit(26);
Cn_dr   = xfit(27);
Cn_da   = xfit(28);

% no flaps
Cx_df   = 0;
Cz_df   = 0;
Cm_df   = 0;

Cy0         = 0;    % zero for symmetric AC
Cl0         = 0;    % zero for symmetric AC
Cn0         = 0;    % zero for symmetric AC