%% Aerodynamic Coeffiecients Identification using lsqcurvefit()
% This script calculates the aerodynamic coefficients (vector "x" 13x1) 
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
%       -   CD_alpha:       d(CD)/d(alpha)
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
% ----------------------------------------------------------------------- %

%% Ajustment of vector sizes 
% WARNING: 
% Do not run this section multiple times (only 1 run)


% run ratesDerivatives.m  % computes pdot, qdot and rdot,
                        % and filter the gyro signals p,q and r

 load simulationData
 load fullStateData.mat


%% Build the DATA MATRICES.
format short

% Measured independent-terms of the moment coeff.
% qS = 0.5.*rho.*Va.*Va.*S;  % Dynamic pressure * Surface

% In the case of the V2 version, uncomment this line
% [~,Torque]=propulsiveModel(Va,deltat,propulsiveParameters);

% For the V3 SIMPLIFIED VERSION, uncomment this line
% Torque = 0;

% Cl_meas = (Jx./(qS.*b)).*( pdot +((Jz-Jy)/Jx).*q.*r - Jxz.*(p.*q+rdot)./Jx)...
%             - Torque./(qS.*b);
% Cm_meas = (Jy./(qS.*c_bar)).*( qdot +((Jx-Jz)/Jy).*p.*r + Jxz.*(p.^2-r.^2)./Jy);
% Cn_meas = (Jz./(qS.*b)).*( rdot +((Jy-Jx)/Jz).*q.*p + Jxy.*(r.*q-pdot)./Jz);

% lb      = -inf.*ones(27,1);
% ub      = inf.*ones(27,1);

% Signs for stable aircrafts
% forces
% lb(1) = 0; % CDp        positive
% lb(2) = 0; % CD_alpha   positive
% lb(4) = 0; % CD_deltae  positive
% lb(7) = 0; % CLq        positive


% lsqcurvefit() setup
x0      = zeros(13,1);
XDATA   = [Va,beta,alpha,p_meas,q_meas,r_meas,rho,deltaa,deltae,deltar,deltat];
YDATA   = [accX,accY,accZ];

% options = optimoptions('lsqcurvefit','TolFun',1e-16);

% minutes = 1000/60;
% Ts      = 0.01;
% Fs      = 1/Ts;

%% nonlinear regression function
% x = lsqcurvefit(@FUN,x0,XDATA,YDATA,lb,ub,options);
xfit = lsqcurvefit(@acc_FUN_v3,x0,XDATA,YDATA);

%% Results
% table of results  
xreal = [0.0193,...
        0.0987,...
        0,...
        0.0135,...
        0.23,...
        5.61,...
        7.65,...
        0.13,...
        -0.83,...
        0,...
        0,...
        0.075,...
        0.19]';

% Data cleaning
minValCoef = min(abs(xreal(xreal~=0))); 
xfit(abs(xfit)<minValCoef) = 0;       % Round small values to zero

     
coeffName = {'1. CD0';...
            '2. CD_alpha';...
            '3. CDq';...
            '4. CD_deltae';...
            '5. CL0';...
            '6. CL_alpha';...
            '7. CLq';...
            '8. CL_deltae';...
            '9. Cy_beta';...
            '10 Cy_p';...
            '11. Cy_r';...
            '12. Cy_da';...
            '13. Cy_dr'};
        xreal_zeros = xreal;
        xreal_zeros(xreal_zeros==0)=1;
 relErrorPorcentage=zeros(13,1);
 for i=1:13
     if (xfit(i)==0 && xreal(i)==0)
         relErrorPorcentage(i) = 0;
     else
         relErrorPorcentage(i) = round(100*abs((xreal_zeros(i)-xfit(i))./xreal_zeros(i)));
     end
 end


T = table(xfit,xreal,relErrorPorcentage,'RowNames',coeffName);
disp(T)



% scatter plot
coeff = linspace(1,13,13);
markerSize = 200;
figure, scatter(coeff,xreal,markerSize*ones(13,1),'x'), hold on, scatter(coeff,xfit), grid minor
legend('REAL COEFF','FITTED COEFF')
xlabel('Nro del Coeficiente')
ylabel('Valor del Coeficiente')

% percentage of error plot
% relErrorPorcentage(relErrorPorcentage>100) = 100;
% figure, scatter(coeff,relErrorPorcentage,markerSize*ones(27,1),'x'),grid minor
% title('PERCENTAGE OF ERROR')
% xlabel('Nro del Coef')
% ylim([0,100])
% save results
save fittedCoefficients xfit;


%% Auxiliary functions

function vecResized = myResize(vec)
    aux         = vec;
    aux(end)    = []; % delete the last component
    aux(1)      = []; % delete the firts component
    vecResized  = aux;
end