%% Aerodynamic Coeffiecients Identification using lsqcurvefit()
% This script calculates the aerodynamic coefficients (vector "x" 27x1) 
% of an airplane model (vectorial function "FUN" 6xN) based on the measured
% state data (matrix "XDATA" MxN) and the accelerometer ouput data 
% (matrix "YDATA" 6xN), by using Least Squares Fitting, lsqcurvefit().
%
% ----------------------------------------------------------------------- %
% 
% In our case exist 7 state variables & 4 input 
% variables to be measured or estimated (XDATA):
% 
%       -   Va:             Airspeed        (m/s)
%       -   beta:           Sideslip angles (rad)
%       -   alpha:          AOA             (rad)
%       -   p:              roll rate       (rad/s) 
%       -   q:              Pitch rate      (rad/s)
%       -   r:              yaw rate        (rad/s)
%       -   rho:            air density     (kg/m3)
%       ...........................................
%       -   deltaa:         Aileron angle   (rad)
%       -   deltae:         Elevator angle  (rad)
%       -   deltar:         Rudder angle    (rad)
%       -   deltat:         Thrust command  (-)
% 
% ----------------------------------------------------------------------- %
% Besides the rates (p,q and r) derivatives, the accelerations measured by 
% the accelerometer are also used to build YDATA matrix:
% 
%       -   accX:       non-gravitational acceleration in Xb axis (m/s2)
%       -   accY:       non-gravitational acceleration in Yb axis (m/s2)
%       -   accZ:       non-gravitational acceleration in Zb axis (m/s2)
% 
% ----------------------------------------------------------------------- %
% 
% The FIRST PART of the coefficients vector corresponds with the FORCE
% COEFF. It has 13 components.
%       -   CDp:            Parasite drag
%       -   CD_alpha:       d(CD)/d(AOA)
%       -   CDq:            damping drag due to q rate
%       -   CD_deltae:      d(CD)/d(deltae)
%       .................................................
%       -   CL0:            zero-AOA lift coeff.
%       -   CL_alpha:       d(CL)/d(AOA)
%       -   CLq:            damping lift due to q rate
%       -   CL_deltae:      d(CL)/d(deltae)
%       .................................................
%       -   Cy_beta:        d(Cy)/d(beta)
%       -   Cy_p:           d(Cy)/d(p)
%       -   Cy_r:           d(Cy)/d(p)
%       -   Cy_da:          d(Cy)/d(delta)
%       -   Cy_dr:          d(Cy)/d(deltar)
% 
% The SECOND PART of the coefficients vector corresponds with the MOMENT
% COEFF. It has 14 components.
%       -   Cl_beta:        roll static stability derivative < 0
%       -   Cl_p:           roll damping derivative
%       -   Cl_r:           d(Cl)/d(r)
%       -   Cl_dr:          cross-control derivative of yaw
%       -   Cl_da:          primary roll control derivative
%       .................................................
%       -   Cm0:            zero-AOA pitching moment coeff.
%       -   Cm_alpha:       longitudinal static stability derivative
%       -   Cm_q:           pitch damping derivative
%       -   Cm_de:          primary pitch control derivative
%       .................................................
%       -   Cn_beta:        yaw static stability derivative > 0
%       -   Cn_p:           d(Cn)/d(p)
%       -   Cn_r:           yaw damping derivative
%       -   Cn_dr:          primary yaw control derivative
%       -   Cn_da:          cross-control derivative of yaw
% 
% In total, the "x" vector has 13+14=27 coefficients.
% 
% ----------------------------------------------------------------------- %

%% Ajustment of vector sizes 
% WARNING: 
% Do not run this section multiple times (only 1 run)

run ratesDerivatives.m  % computes pdot, qdot and rdot,
                        % and filter the gyro signals p,q and r

load simulationData
Va      = myResize(Va);
beta    = myResize(beta);
alpha   = myResize(alpha);
p       = myResize(p);
q       = myResize(q);
r       = myResize(r);
rho     = myResize(rho);
deltaa  = myResize(deltaa);
deltae  = myResize(deltae);
deltar  = myResize(deltar);
deltat  = myResize(deltat);

accX    = myResize(accX);
accY    = myResize(accY);
accZ    = myResize(accZ);

%% Build the DATA MATRICES.
format short

% Measured independent-terms of the moment coeff.
qS = 0.5.*rho.*Va.*Va.*S;  % Dynamic pressure * Surface

Cl_meas = (Jx./(qS.*b)).*( pdot +((Jz-Jy)/Jx).*q.*r - Jxz.*(p.*q+rdot)./Jx);
Cm_meas = (Jy./(qS.*c_bar)).*( qdot +((Jx-Jz)/Jy).*p.*r + Jxz.*(p.^2-r.^2)./Jy);
Cn_meas = (Jz./(qS.*b)).*( rdot +((Jy-Jx)/Jz).*q.*p + Jxy.*(r.*q-pdot)./Jz);

lb      = -inf.*ones(27,1);
ub      = inf.*ones(27,1);

ub((end-3):end)=[0 0 0 0];
 
% lsqcurvefit() setup
x0      = rand(27,1).*ones(27,1);
XDATA   = [Va,beta,alpha,p,q,r,rho,deltaa,deltae,deltar,deltat];
YDATA   = [accX,accY,accZ,Cl_meas,Cm_meas,Cn_meas];

options = optimoptions('lsqcurvefit','TolFun',1e-16);

% minutes = 1000/60;
% Ts      = 0.01;
% Fs      = 1/Ts;

%% nonlinear regression function
% x = lsqcurvefit(@FUN,x0,XDATA,YDATA,lb,ub,options);
xfit = lsqcurvefit(@FUN,x0,XDATA,YDATA,lb,ub,options);

%% Results
% table of results  
xreal = [0.03,0.030,0,0.0135,0.23,5.61,7.65,0.13,-0.83,0,0,0.075,0.19...
         -0.13, -0.51, 0.25, 0.0024, 0.17, 0.0135, -2.74, -38.21, -0.99, 0.073, -0.069, -0.095, -0.069, -0.011]';
coeffName = {'CDp';...
            'CD_alpha';...
            'CDq';...
            'CD_deltae';...
            'CL0';...
            'CL_alpha';...
            'CLq';...
            'CL_deltae';...
            'Cy_beta';...
            'Cy_p';...
            'Cy_r';...
            'Cy_da';...
            'Cy_dr';...
            'Cl_beta';...
            'Cl_p';...
            'Cl_r';...    
            'Cl_dr';...   
            'Cl_da';...   
            'Cm0';...    
            'Cm_alpha';...
            'Cm_q';...    
            'Cm_de';...   
            'Cn_beta';... 
            'Cn_p';...    
            'Cn_r';...    
            'Cn_dr';...   
            'Cn_da'};
T = table(xfit,xreal,'RowNames',coeffName);
disp(T)

% scatter plot
coeff = linspace(1,27,27);
markerSize = 200;
figure, scatter(coeff,xreal,markerSize*ones(27,1),'x'), hold on, scatter(coeff,xfit), grid minor
legend('REAL COEFF','FITTED COEFF')
xlabel('Nro del Coeficiente')
ylabel('Valor del Coeficiente')

% save results
save fittedCoefficients xfit;


%% Auxiliary functions

function vecResized = myResize(vec)
    aux         = vec;
    aux(end)    = []; % delete the last component
    aux(1)      = []; % delete the firts component
    vecResized  = aux;
end