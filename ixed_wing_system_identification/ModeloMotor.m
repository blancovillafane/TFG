    RPM     = @(deltat)9000.*tanh(0.5.*(9.*deltat-4))+9000;
        
    deltat_pts  = 0.1.* [0:10/6:10];
    RPM_pts     = RPM(deltat_pts);
      
    
    plot(deltat_pts,RPM_pts), hold on, fplot(RPM), xlim([0 1]),ylim([0 18000])