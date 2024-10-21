format short
b1=1;
b2=2;

x0      = [1 1]';
lb      = [-1 -1]';
ub      = [2 3]';
XDATA   = [x1,x2];
YDATA   = [fx,fy,fz];
minutes = 1000/60;
Ts      = 0.01;
Fs      = 1/Ts;
options = optimoptions('lsqcurvefit','TolFun',1e-12);
%% nonlinear regression function
x = lsqcurvefit(@FUN,x0,XDATA,YDATA,lb,ub,options);

