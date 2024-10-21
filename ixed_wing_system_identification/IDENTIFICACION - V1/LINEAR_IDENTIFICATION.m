%% Aerodynamic Coeffiecients Identification using SVD
% This script calculates the aerodynamic coefficients of an airplane
% (vector b) based on flight data (vector a), by computing the SVD
% factorization of the matrix of candidate funtions Theta.
% 
% Solving the system: Theta*b=a
% 
% Theta     = USV'
% Thetaplus = V*inv(S)*U'   % pseudo-inverse of Theta
% 
% Finally: b = Thetaplus*a
%
% In our case, Theta only depends on the following state variables & input 
% variables:
%       DINAMICA LONGITUDINAL
%       -   alpha:          AOA             (rad)
%       -   q:              Pitch rate      (rad/s)
%       -   Va:             Airspeed        (m/s)
%       -   deltae:         Elevator angle  (rad)
%       DINAMICA LATERAL
%       -   beta:           Sideslip angles (rad)
%       -   p:              roll rate       (rad/s)    
%       -   r:              yaw rate        (rad/s)
%       -   deltaa:         Aileron angle   (rad)
%       -   deltar:         Rudder angle    (rad)
% 
%% Build the Theta matrix. Auxiliar Vectors
qbar=0.5.*Va.*Va*1.225;
mu1 = c_bar.*q./(2.*Va);
% mu2 = b./(2.*Va);
sa  = sin(alpha);
ca  = cos(alpha);
%%
% The columns of the matrix ThetaX and ThetaZ, recieve the names nu_i
nu1 = sa;
nu2 = sa.*alpha;
nu3 = sa.*mu1;
nu4 = sa.*deltae;
nu5 = ca;
nu6 = ca.*alpha;
nu7 = ca.*mu1;
nu8 = ca.*deltae;
%%
ThetaZ      = -qbar.*[nu1,nu2,nu3,nu4,nu5,nu6,nu7,nu8];
[Uz,Sz,Vz]  = svd(ThetaZ,'econ');
ThetaZplus  = Vz*(Sz\Uz');
% ThetaZplus  = Vz*inv(Sz)*Uz';
%%
ThetaX      = [-nu5,-nu6,-nu7,-nu8,nu1,nu2,nu3,nu4];
[Ux,Sx,Vx]  = svd(ThetaX,'econ');
ThetaXplus  = Vx*(Sx\Ux');
% ThetaXplus  = Vx*inv(Sx)*Ux';

%%
% The columns of the matrix ThetaY, recieve the name theta_i
theta1 = beta;
theta2 = mu2.*p; 
theta3 = mu2.*r; 
theta4 = deltaa; 
theta5 = deltar; 

ThetaY      = [theta1,theta2,theta3,theta4,theta5];
[Uy,Sy,Vy]  = svd(ThetaY,'econ');
ThetaYplus  = Vy*(Sy\Uy');
% ThetaYplus  = Vy*inv(Sy)*Uy';

%% Solution
b = ThetaZ\(accZ.*mass./S)
b = ThetaZplus*(accZ.*mass./S)
b = ThetaX\(accZ.*mass./S)
b = ThetaXplus*(accZ.*mass./S)
b = (ThetaX*5+ThetaZ*2)\((accX*5+accZ*2).*mass./S)