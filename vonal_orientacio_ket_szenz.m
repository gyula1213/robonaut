function [delta, p1, p2] = vonal_orientacio_ket_szenz(e,p1,p2,no_sensors,delta,e_b,senz_dist, dist_sensors)
   
    for i=1:no_sensors
        if (e(i)>0.6)        
            p1 = i;          
        end
        if (e_b(i)>0.6)   
            p2 = i;       
        end
    end 
     dist = (p2-p1)*dist_sensors;
     delta = atan((dist)/senz_dist);
     delta_fok = delta/pi*180;
end