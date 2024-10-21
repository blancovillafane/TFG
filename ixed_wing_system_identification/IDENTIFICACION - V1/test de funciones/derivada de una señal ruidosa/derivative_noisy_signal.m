%% filter setup
windowSize = 80;
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y = filtfilt(b,a,x1);

%% x1 VS y = LPF(x1) plots
close, figure
plot(tout,x1,'c','LineWidth',.01)
hold on
plot(tout,y,'-.r','LineWidth',3)
grid minor
xlim([70,90])

%% ydot = d(y)/dt
% "finite difference" derivative. 
% Note it is one element shorter than y and x
ydot = diff(y)./diff(tout);
% this is to assign yd an abscissa midway between two subsequent t
tdot = (tout(2:end)+tout(1:(end-1)))/2;
% this should be a rough plot of your derivative
plot(tdot,ydot,'m','LineWidth',0.01)

%% LPF(ydot)
ydfiltered = filtfilt(b,a,ydot);
hold on 
plot(td,ydfiltered,'-.b','LineWidth',3)
%% styling

legend('COS MEASURED','LPF(COS MEAS.)','D(LPF(COS MEAS))/Dt','LPF(D(LPF(COS MEAS))/Dt)')