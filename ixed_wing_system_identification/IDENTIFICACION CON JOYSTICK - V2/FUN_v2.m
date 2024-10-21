function [YDATA] = FUN_v2(x,XDATA)
% FUN: implements de nonlinear function that models 
% the aerodynamic forces and moments of the aircraft, as a vectorial
% function of the vector of parameters, X (coeff. vector) and the state
% matrix of the A/C, XDATA (state measurements).

%% parameters of the A/C
c_bar = 0.19;
b = 2.9;
S = 0.55;
mass = 11;
load propulsiveParameters.mat propulsiveParameters

%% x vector translation
% forces
CDp         = x(1);
k2          = x(2);
k1          = x(3);
CDq         = x(4);
CD_deltae   = x(5);

CL0         = x(6);
CL_alpha    = x(7);
CLq         = x(8);
CL_deltae   = x(9);

Cy_beta = x(10);
Cy_p    = x(11);
Cy_r    = x(12);
Cy_da   = x(13);
Cy_dr   = x(14);

% moments
Cl_beta = x(15);   
Cl_p    = x(16);  
Cl_r    = x(17);  
Cl_dr   = x(18); 
Cl_da   = x(19);

Cm0         = x(20);
Cm_alpha    = x(21);
Cm_q        = x(22);
Cm_de       = x(23);

Cn_beta = x(24);
Cn_p    = x(25);
Cn_r    = x(26);
Cn_dr   = x(27);
Cn_da   = x(28);

%% XDATA matrix translation
Va      = XDATA(:,1);
beta    = XDATA(:,2);
alpha   = XDATA(:,3);

p       = XDATA(:,4);
q       = XDATA(:,5);
r       = XDATA(:,6);

rho  = XDATA(:,7);

deltaa  = XDATA(:,8);
deltae  = XDATA(:,9);
deltar  = XDATA(:,10);
deltat  = XDATA(:,11);

%% Formulas
% Aerodynamic coeff.
CL = CL0 + CL_alpha.*alpha;
CD = CDp + k2.*CL + k1.*CL.*CL;

% trigonometric functions of AOA
ca = cos(alpha);
sa = sin(alpha);

% Force Coeff.
Cx = (-CD.*ca+CL.*sa) + (-CDq.*ca+CLq.*sa).*(c_bar.*q.*0.5./Va) + ...
     (-CD_deltae.*ca+CL_deltae.*sa).*deltae;
Cy = Cy_beta.*beta + Cy_p.*p.*0.5.*b./Va + Cy_r.*r.*0.5.*b./Va + ...
     Cy_da.*deltaa + Cy_dr.*deltar;
Cz = (-sa.*CD-ca.*CL) + (-sa.*CDq-ca.*CLq).*(c_bar.*q.*0.5./Va) + ...
     (-sa.*CD_deltae-ca.*CL_deltae).*deltae;

% Auxiliary variables
a0          = rho.*Va.*Va.*S./(2.*mass);
[Thrust,~]  = propulsiveModel(Va,deltat,propulsiveParameters);

% Acceletometer model
accX  = a0.*Cx + Thrust/mass; 
accY  = a0.*Cy; 
accZ  = a0.*Cz;

% Moment coeff.
Cl = beta.*Cl_beta + Cl_p.*p.*b./(2.*Va) + Cl_r.*r.*b./(2.*Va) +...
     Cl_dr.*deltar + Cl_da.*deltaa;
Cm = Cm0 + Cm_alpha.*alpha + Cm_q.*c_bar.*q./(2.*Va) + Cm_de.*deltae;
Cn = beta.*Cn_beta + Cn_p.*p.*b./(2.*Va) + Cn_r.*r.*b./(2.*Va) +...
     Cn_dr.*deltar + Cn_da.*deltaa;

% WE ARE ASSUMING THAT THE A/C IS SYMMETRIC: Cl0 = Cn0 = Cy0 = 0

%% FUN Output
YDATA = [accX,accY,accZ,Cl,Cm,Cn];
end

