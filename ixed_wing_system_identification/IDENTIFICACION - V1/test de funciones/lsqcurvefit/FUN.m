function [YDATA] = FUN(x,XDATA)

b1=x(1);
b2=x(2);

x1=XDATA(:,1);
x2=XDATA(:,2);

fx = b1.*x1     +   b2.*x2.^2;
fy = b1.*x1.^3  -   sin(x2)./b2;   
fz = b2.*x2     +   b1.*exp(x1);

YDATA = [fx,fy,fz];

end

