function [YDATA] = FUN(x,XDATA)
% FUN: implements de nonlinear function that models 
% the aerodynamic forces and moments of the aircraft, as a vectorial
% function of the vector of parameters, X (coeff. vector) and the state
% matrix of the A/C, XDATA (state measurements).

%% parameters of the A/C
c_bar = 0.19;
b = 2.9;
C_prop = 1;
k_motor = 80;
Sprop = 0.2027;
S = 0.55;
mass = 11;

%% x vector translation
% forces
CDp         = x(1);
CD_alpha    = x(2);
CDq         = x(3);
CD_deltae   = x(4);

CL0         = x(5);
CL_alpha    = x(6);
CLq         = x(7);
CL_deltae   = x(8);

Cy_beta = x(9);
Cy_p    = x(10);
Cy_r    = x(11);
Cy_da   = x(12);
Cy_dr   = x(13);

% moments
Cl_beta = x(14);   
Cl_p    = x(15);  
Cl_r    = x(16);  
Cl_dr   = x(17); 
Cl_da   = x(18);

Cm0         = x(19);
Cm_alpha    = x(20);
Cm_q        = x(21);
Cm_de       = x(22);

Cn_beta = x(23);
Cn_p    = x(24);
Cn_r    = x(25);
Cn_dr   = x(26);
Cn_da   = x(27);

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
CD = CDp + CD_alpha.*alpha;

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
a0      = rho.*Va.*Va.*S./(2.*mass);
aprop   = rho.*Sprop.*C_prop./(2.*mass);

% Acceletometer model
accX  = a0.*Cx + aprop.*((k_motor.*deltat).^2-Va.^2); 
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

