%% CONFIGURATION SCRIPT
% 
%  This script sets for default the parameters of the Aerosonde UAV. These
%  parameters should be ajusted in order to simulate the desired UAV.
% 
%  All variables most be expressed in SI UNITS.
% 
%  The Aerosonde UAV parameters are reported in the book: "Small Unmanned
%  Aircraft - Beard&McLain".
% 
%% Load Propulsion Data
run('LOOKUP_TABLES.m');
load fitted_coefficients;

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
Jxy     = 0;
J       = [  Jx,    -Jxy,       -Jxz;...
           -Jxy,     Jy,        -Jxy;...
           -Jxz,    -Jxy,        Jz];     % Inertia Tensor
       
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

Sprop = 0.2027;
rho     = 1.2682;
k_motor = 80;
k_Tp    = 0;
k_omega = 0;
C_prop  = 1;
%% Aerodynamics

% forces
CDp         = xfit(1);
CD_alpha    = xfit(2);
CDq         = xfit(3);
CD_deltae   = xfit(4);

CL0         = xfit(5);
CL_alpha    = xfit(6);
CLq         = xfit(7);
CL_deltae   = xfit(8);

Cy_beta = xfit(9);
Cy_p    = xfit(10);
Cy_r    = xfit(11);
Cy_da   = xfit(12);
Cy_dr   = xfit(13);

% moments
Cl_beta = xfit(14);   
Cl_p    = xfit(15);  
Cl_r    = xfit(16);  
Cl_dr   = xfit(17); 
Cl_da   = xfit(18);

Cm0         = xfit(19);
Cm_alpha    = xfit(20);
Cm_q        = xfit(21);
Cm_de       = xfit(22);

Cn_beta = xfit(23);
Cn_p    = xfit(24);
Cn_r    = xfit(25);
Cn_dr   = xfit(26);
Cn_da   = xfit(27);