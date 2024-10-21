load simulationData.mat
load fullStateData.mat

length(time)
length(q)
length(q_meas)
plot(time,(q_meas),'b--')
hold on
plot(time,q,'r-', 'LineWidth',1)
title('q')

figure

length(time)
length(p)
length(p_meas)
plot(time,(p_meas),'b--')
hold on
plot(time,p,'r-', 'LineWidth',1)
title('p')

figure

length(time)
length(r)
length(r_meas)
plot(time,(r_meas),'b--')
hold on
plot(time,r,'r-', 'LineWidth',1)
title('r')


figure

length(time)
V=sqrt(u.*u+v.*v+w.*w);
length(V)
length(Va)
plot(time,(Va),'b--')
hold on
plot(time,V,'r-', 'LineWidth',1)
title('Va')

figure

length(time)
alphaclean=atan(w./u);
length(alphaclean)
length(alpha)
plot(time,(alpha),'b--')
hold on
plot(time,alphaclean,'r-', 'LineWidth',1)
title('alpha')

figure

length(time)
betaclean=atan(v./u);
length(betaclean)
length(beta)
plot(time,(beta),'b--')
hold on
plot(time,betaclean,'r-', 'LineWidth',1)
title('beta')

function vecResized = myResize(vec)
aux         = vec;
aux(end)    = []; % delete the last component
aux(1)      = []; % delete the firts component
vecResized  = aux;
end
