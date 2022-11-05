function [delta, p, s] = vonal_orientacio_szamitas_dif(e,p,no_sensors,u,dT,delta,p_e, s)

    s = s + u*dT;
    for i=1:no_sensors
        if (e(i)>0.6)   
            p_prev = p;
            p = p_e(1,i); 
            if (p~=p_prev)
                delta = atan((p_prev-p)/(s));
                delta_fok = delta/pi*180;
                s = 0;
            end   
        end
    end 
end