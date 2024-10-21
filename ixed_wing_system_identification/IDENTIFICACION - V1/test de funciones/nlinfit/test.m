format longe
b1=1;
b2=2;
%% 
cos_col= cos(alpha);
alphasquared_col= alpha.^2;
% a=lowpass(a,
THETA = [cos_col,alphasquared_col];
[Uhat,Shat,V] = svd(THETA, 'econ');

THETA_plus = V*(Shat\Uhat');

[Q,R]=qr(THETA,0);

THETA_plus*a
THETA\a
R\(Q'*a)
%% nonlinear regression function
beta0=[0 0]';
X = [alpha];
Y = a;
fun = @(beta,alpha) cos(alpha).*beta(1)+beta(2).*alpha.^2;

betas = nlinfit(X,Y,fun,beta0) 

