function [delta, p, s, p_start] = vonal_orientacio_szamitas(e,p,no_sensors,u,dT,delta,p_e, s,p_start)
    if (p == 0)
        for i=1:no_sensors
            if (e(i)>0.6)
                p_start = p_e(1,i);
                s = 0;
            end
        end
    end
    s = s + u*dT;
   
    for i=1:no_sensors
        if (e(i)>0.6)   
            p = p_e(1,i);  
            delta = atan((p_start-p)/(s));
            delta_fok = delta/pi*180;
        end
    end 
    
end