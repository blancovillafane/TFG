%% This scripts compute the derivatives (vectors) of the signals p,q and r
% The process consists of the following steps:
%   1. clean_signal         = LPF(signal)
%   2. noisy_derivative     = d(clean_signal)/dt 
%   3. signal_derivative    = LPF(noisy_derivative)
% Where "signal" is any vector in the set {p,q,r} and LPF() is a Low Pass
% Filter Function with zero-phase to prevent delays.
% 
% 
% 
% --------------------------------------------------------------------- %
% ---------------------------    q RATE    ---------------------------- %
% --------------------------------------------------------------------- %
load simulationData
% filter setup
optimal_windowSize_q = optimalFiltering(q_meas,q,time);
windowSize = optimal_windowSize_q;
bfilt = (1/windowSize)*ones(1,windowSize);
afilt = 1;
q_filt = filtfilt(bfilt,afilt,q_meas);
q = q_meas;

% q_meas VS q = LPF(q_meas) plots 
    % figure
    % plot(tout,q_meas,'c','LineWidth',.01) 
    % hold on
    % plot(tout,q,'-.b','LineWidth',3)
    % hold on
    % plot(tout,q_filt,'-.r','LineWidth',3)
    % grid minor
    % title('q rate signal filtering')
    % legend('Measured q','Real q','Filtered q')

% "finite difference" derivative. 
% Note it is one element shorter than y and x
qdot = diff(q_filt)./diff(time);
% this is to assign yd an abscissa midway between two subsequent t
tdot = (time(2:end)+time(1:(end-1)))/2;

    % this should be a rough plot of your derivative
    % plot(tdot,qdot,'m','LineWidth',0.01)

windowSize = windowSize + 2;
bfilt = (1/windowSize)*ones(1,windowSize).*1.1;
afilt = 1;
qdot = filtfilt(bfilt,afilt,qdot);
    % hold on 
    % plot(tdot,qdot,'-.b','LineWidth',3)
    % ylim([-2 2])
    % xlim([13 19])

qdot = normalizeLengths(qdot);

% --------------------------------------------------------------------- %
% ---------------------------    p RATE    ---------------------------- %
% --------------------------------------------------------------------- %
%% filter analysis -> FIND THE BEST WINDOW SIZE FOR THE FILTER
optimal_windowSize_p = optimalFiltering(p_meas,p,time);
% filter setup
windowSize = optimal_windowSize_p;
bfilt = (1/windowSize)*ones(1,windowSize);
afilt = 1;
p_filt = filtfilt(bfilt,afilt,p_meas);
p = p_meas;

% r_meas VS r = LPF(r_meas) plots
    % close all
    % figure
    % plot(tout,p_meas,'c','LineWidth',.01) 
    % hold on
    % plot(tout,p,'-.b','LineWidth',3)
    % hold on
    % plot(tout,p_filt,'-.r','LineWidth',3)
    % grid minor
    % title('r rate signal filtering')
    % legend('Measured p','Real p','Filtered p')

% "finite difference" derivative. 
% Note it is one element shorter than y and x
pdot = diff(p_filt)./diff(time);

    % this is to assign yd an abscissa midway between two subsequent t
    % tdot = (tout(2:end)+tout(1:(end-1)))/2;
    % this should be a rough plot of your derivative
    % plot(tdot,pdot,'m','LineWidth',0.01)

windowSize = windowSize +4;
bfilt = (1/windowSize)*ones(1,windowSize).*1.025;
afilt = 1;
pdot = filtfilt(bfilt,afilt,pdot);
    % hold on 
    % plot(tdot,pdot,'-.b','LineWidth',3)
    % ylim([-7 7])
    % xlim([0 10])

pdot = normalizeLengths(pdot);

% --------------------------------------------------------------------- %
% ---------------------------    r RATE    ---------------------------- %
% --------------------------------------------------------------------- %
%% filter analysis -> FIND THE BEST WINDOW SIZE FOR THE FILTER
optimal_windowSize_r = optimalFiltering(r_meas,r,time);
windowSize = optimal_windowSize_r;
bfilt = (1/windowSize)*ones(1,windowSize);
afilt = 1;
r_filt = filtfilt(bfilt,afilt,r_meas);
r = r_meas;

% r_meas VS r = LPF(r_meas) plots
    % close all
    % figure
    % plot(tout,r_meas,'c','LineWidth',.01) 
    % hold on
    % plot(tout,r,'-.b','LineWidth',3)
    % hold on
    % plot(tout,r_filt,'-.r','LineWidth',3)
    % grid minor
    % title('r rate signal filtering')
    % legend('Measured r','Real r','Filtered r')

% "finite difference" derivative. 
% Note it is one element shorter than y and x
rdot = diff(p_filt)./diff(time);

    % this is to assign yd an abscissa midway between two subsequent t
    % tdot = (tout(2:end)+tout(1:(end-1)))/2;
    % this should be a rough plot of your derivative
    % plot(tdot,rdot,'m','LineWidth',0.01)

windowSize = windowSize +4;
bfilt = (1/windowSize)*ones(1,windowSize).*1.01;
afilt = 1;
rdot = filtfilt(bfilt,afilt,rdot);

    % hold on 
    % plot(tdot,rdot,'-.b','LineWidth',3)
    % ylim([-7 7])
    % xlim([0 10])
    % length(rdot)

rdot = normalizeLengths(rdot);

    % length(rdot)
    % hold on
    % tnorm = tout;
    % tnorm(end) = [];
    % tnorm(1)   = [];
    % length(tnorm)
    % plot(tnorm,rdot,'-.g','LineWidth',2.5)


%% AUXILIARY FUNCTIONS
function optimal_windowSize = optimalFiltering(signal_meas,signal,tout)
% Filter Analysis -> FINDS THE BEST WINDOW SIZE FOR THE FILTER
% Computes the error between the real signal and its noisy version after
% filtering, and returns "optimal_windowSize" for the filter such that
% the error is minimum.

means_vector = zeros(10,1);
k=1;
for i = 1:2:20    
    % filter setup
    windowSize = i;
    b = (1/windowSize)*ones(1,windowSize);
    a = 1;
    signal_filt = filtfilt(b,a,signal_meas);
    % error plot
    error_signal = 100*abs(signal-signal_filt)/max(signal);
    means_vector(k) = mean(error_signal);
    k=k+1;
end

% ERROR VS WINDOW SIZE PLOT
windowSizes = 1:2:20;
signalName = inputname(2);

    % figure, plot(windowSizes,means_vector);
    % 
    % xlabel('window size')
    % ylabel("average error between "+signalName+" and filtered "+signalName)


optimal_windowSize_index = find(means_vector==min(means_vector));
optimal_windowSize = windowSizes(optimal_windowSize_index);
minimum_errorMean = means_vector(optimal_windowSize_index);

% ERROR(t) and mean(ERROR)(t) PLOT
% optimal filter setup
windowSize = optimal_windowSize;
b = (1/windowSize)*ones(1,windowSize);
a = 1;
signal_filt = filtfilt(b,a,signal_meas);                % filtering
error_signal = 100*abs(signal-signal_filt)/max(signal); % error 
    
    % figure, plot(tout,error_signal);                        % plot error
    % 
    % mean_error = mean(error_signal)*ones(length(error_signal),1);
    % mytitle = "error in the estimated "+ signalName +...
    %           "  - windowSize = "+num2str(optimal_windowSize);
    % title(mytitle)
    % hold on, plot(tout,mean_error)                          % plot mean error
    % ylim([0,100])

end

function normalSignal = normalizeLengths(signalDerivative)
% Normalize the pdot, qdot and rdot sizes to N-2. N = length(p,q or r)
Nminus1 = length(signalDerivative);
j=1;
normalSignal = zeros(Nminus1-1,1);
for l = 1:(Nminus1-1)
    linspace_vector = linspace( signalDerivative(j+1), signalDerivative(j), 3);  
    normalSignal(l) = linspace_vector(2);
    j=j+1;
end

end