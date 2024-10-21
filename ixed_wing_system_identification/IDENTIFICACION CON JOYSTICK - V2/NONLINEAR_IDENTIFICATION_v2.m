%% Aerodynamic Coeffiecients Identification using lsqcurvefit()
% This script calculates the aerodynamic coefficients (vector "x" 28x1) 
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
% COEFF. It has 14 components.
%       -   CDp:            Parasite drag
%       -   k2:             d(CD)/d(CL)
%       -   k1:             d(CD)/d(CL^2)
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
% In total, the "x" vector has 14+14=28 coefficients.
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
p       = myResize(p_meas);
q       = myResize(q_meas);
r       = myResize(r_meas);
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

[~,Torque]=propulsiveModel(Va,deltat,propulsiveParameters);
% Torque = zeros(length(Va),1);
% for i=1:length(Va)
%     [~,B]=propulsiveModel(Va(i),deltat(i),propulsiveParameters);
%     Torque(i) = B;
%     progress=(i*100)/length(Va);
%     disp("Progress: "+num2str(progress))
% end

Cl_meas = (Jx./(qS.*b)).*( pdot +((Jz-Jy)/Jx).*q.*r - Jxz.*(p.*q+rdot)./Jx)...
            - Torque./(qS.*b);
Cm_meas = (Jy./(qS.*c_bar)).*( qdot +((Jx-Jz)/Jy).*p.*r + Jxz.*(p.^2-r.^2)./Jy);
Cn_meas = (Jz./(qS.*b)).*( rdot +((Jy-Jx)/Jz).*q.*p + Jxy.*(r.*q-pdot)./Jz);

lb      = -inf.*ones(28,1);
ub      = inf.*ones(28,1);
ub(2)   = 0; % k2         negative
ub(22)  = 0; % Cm_q       negative
lb(1)   = 0; % CDp        positive
lb(8)   = 0; % CLq        positive
lb(3)   = 0; % k1         positive
lb(5)   = 0; % CD_deltae  positive


% lsqcurvefit() setup
x0      = rand(28,1).*ones(28,1);
XDATA   = [Va,beta,alpha,p,q,r,rho,deltaa,deltae,deltar,deltat];
YDATA   = [accX,accY,accZ,Cl_meas,Cm_meas,Cn_meas];

options = optimoptions('lsqcurvefit','TolFun',1e-16);

% minutes = 1000/60;
% Ts      = 0.01;
% Fs      = 1/Ts;

%% nonlinear regression function
% x = lsqcurvefit(@FUN,x0,XDATA,YDATA,lb,ub,options);
xfit = lsqcurvefit(@FUN_v2,x0,XDATA,YDATA,lb,ub,options);
length(xfit)
%% Results
% table of results  
xreal = [0.0234,-0.0241,0.0537,0,0.0135,0.23,5.61,7.65,0.13,-0.83,0,0,0.075,0.19...
         -0.13, -0.51, 0.25, 0.0024, 0.17, 0.0135, -2.74, -38.21, -0.99, 0.073, -0.069, -0.095, -0.069, -0.011]';

% Data cleaning
minValCoef = min(abs(xreal(xreal~=0))); 
xfit(abs(xfit)<minValCoef) = 0;       % Round small values to zero

     
coeffName = {'1. CDp';...
            '2. k2';...
            '3. k1';...
            '4. CDq';...
            '5. CD_deltae';...
            '6. CL0';...
            '7. CL_alpha';...
            '8. CLq';...
            '9. CL_deltae';...
            '10. Cy_beta';...
            '11. Cy_p';...
            '12. Cy_r';...
            '13. Cy_da';...
            '14. Cy_dr';...
            '15. Cl_beta';...
            '16. Cl_p';...
            '17. Cl_r';...    
            '18. Cl_dr';...   
            '19. Cl_da';...   
            '20. Cm0';...    
            '21. Cm_alpha';...
            '22. Cm_q';...    
            '23. Cm_de';...   
            '24. Cn_beta';... 
            '25. Cn_p';...    
            '26. Cn_r';...    
            '27. Cn_dr';...   
            '28. Cn_da'};
        xreal_zeros = xreal;
        xreal_zeros(xreal_zeros==0)=1;
 
 for i=1:28
     if (xfit(i)==0 && xreal(i)==0)
         relErrorPorcentage(i) = 0;
     else
         relErrorPorcentage(i) = round(100*abs((xreal_zeros(i)-xfit(i))./xreal_zeros(i)));
     end
 end


T = table(xfit,xreal,relErrorPorcentage','RowNames',coeffName);
disp(T)



% scatter plot
coeff = linspace(1,28,28);
markerSize = 200;
figure, scatter(coeff,xreal,markerSize*ones(28,1),'x'), hold on, scatter(coeff,xfit), grid minor
legend('REAL COEFF','FITTED COEFF')
xlabel('Nro del Coeficiente')
ylabel('Valor del Coeficiente')

% percentage of error plot
% relErrorPorcentage(relErrorPorcentage>100) = 100;
% figure, scatter(coeff,relErrorPorcentage,markerSize*ones(28,1),'x'),grid minor
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