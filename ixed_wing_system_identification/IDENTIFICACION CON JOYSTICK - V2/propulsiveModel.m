function [Thrust,Torque] = propulsiveModel(Va,deltat,parameters)
%%  COMPLETE PROPULSIVE MODEL FOR THE V2 SIMULATOR. Computes the thrust and
%   torque of the propulsive system of the UAV based on the aerodynamic
%   velocity "Va" and the throttle "deltat".
% 
%   The model parameters are required as inputs. These have to be passed to
%   the function as a structure object with the attributes:
%       - deltat_pts:   interpolation points vector of the throttle signal
%       - RPM_pts:      interpolation points vector of the RPMs
%       - Nmatrix:      interpolation points matrix of the RPM/1000
%       - Vmatrix:      interpolation points matrix of the Va
%       - M:            interpolation points matrix of the torque M
%       - T:            interpolation points matrix of the thrust T
% 
%%   MOTOR MODEL
%   RPM_function    = @(deltat)9000.*tanh(0.5.*(9.*deltat-4))+9000;  
    deltat_pts      = parameters.deltat_pts;
    RPM_pts         = parameters.RPM_pts;
    RPM = interp1(deltat_pts,RPM_pts,deltat); % RPM motor LOOKUP TABLE
    
%   plot(deltat_pts,RPM_pts), hold on, fplot(RPM_function), xlim([0 1]),ylim([0 18000])
 
%%   PROPELLER MODEL
%   load propulsiveParameters.mat propulsiveParameters;
    Nmatrix = parameters.Nmatrix;
    Vmatrix = parameters.Vmatrix;
    M       = parameters.M;
    T       = parameters.T;
    
%%   OUTPUTS
    Thrust = interp2(Nmatrix,Vmatrix,T,RPM/1000,Va);
    Torque = interp2(Nmatrix,Vmatrix,M,RPM/1000,Va);
end
