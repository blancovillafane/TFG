%% CONFIGURATION SCRIPT
% 
%  This script sets the default the parameters of the Simulink Simulators. 
%  These parameters should be ajusted in order to simulate the desired UAV.
% 
%  All variables most be expressed in SI UNITS.
% 
%  The default aircraft has the AerosondeUAV airframe parameters but with
%  the Polar Curves and Lift Curve of the JALEO-1, designed by VANTUS
%  AeroDesign Team from the University of Seville.
% 
%  The Aerosonde UAV parameters are reported in the book: "Small Unmanned
%  Aircraft - Beard&McLain" while the JALEO-1 technical report can be
%  requested to: blancovillafane@gmail.com
% 
%% Load Propulsion Data
run('LOOKUP_TABLES.m');
ST = 0.001;             % SAMPLE TIME
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
CL_cruise       = 0.38797;
alpha_cruise    = 0.0282;

% Stability Axis Aerodynamics (JALEO-1)
k1          = 0.0537;     % d(CD)/d(CL2)
k2          = -0.0241;    % d(CD)/d(CL)
CDp         = 0.0234;     % parasite drag coef.
% The polar formula used: CD = CDp + k1*CL2 + k2*CL

CD0         = 0.0193;     % linear parasite drag coef.
CD_alpha    = 0.0987;     % d(CD)/d(alpha)
% The polar formula used: CD = CD0 + CD_alpha*alpha

% These values of correspond to the JALEO 1 drone from the
% VANTUS AeroTeam of the University of Seville

CDq         = 0;        % damping drag due to q rate
CD_deltae   = 0.0135;   % elevator drag derivative

% ================================== %

CL0         = 0.23;     % zero-AOA lift coef.
CL_alpha    = 5.61;     % lift AOA derivative
CLq         = 7.95;     % lift due to q rate

CL_deltae   = 0.13;     % elevator lift derivative

Cm0         = 0.0135;   % zero-AOA moment m coef.
Cm_alpha    = -2.74;    % moment m-AOA derivative

% ====================================================================== %
% Airflow (alpha-beta) Dependent Derivatives 
Cy_beta     = -0.83;    
Cl_beta     = -0.13;
Cn_beta     = 0.073;

Cy0         = 0;    % zero for symmetric AC
Cl0         = 0;    % zero for symmetric AC
Cn0         = 0;    % zero for symmetric AC

% ====================================================================== %
% Actuator Increments Derivatives
%   da = aleron incremental deflection
    Cy_da       = 0.075;
    Cl_da       = 0.17;
    Cn_da       = -0.011;

%   de = elevator incremental deflection
    % Cx_deltae = f(CL_deltae,CD_deltae,alpha)
    % Cz_deltae = f(CL_deltae,CD_deltae,alpha)
    Cm_de       = -0.99;

%   dr = rudder incremental deflection
    Cx_dr   = 0;
    Cy_dr   = 0.19;    
    Cl_dr   = 0.0024;
    Cn_dr   = -0.069;

%   df = flaps incremental deflection
    Cx_df   = 0;
    Cz_df   = 0;
    Cm_df   = 0;
    
% ====================================================================== %
% Damping Derivatives
Cy_p        = 0;
Cl_p        = -0.51;
Cn_p        = -0.069;

% Cx_q  = f(CL_deltae,CD_deltae,alpha)
% Cz_q  = f(CL_deltae,CD_deltae,alpha)
Cm_q    = -38.21;

Cy_r    = 0;
Cl_r    = 0.25;
Cn_r    = -0.095;