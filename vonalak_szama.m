function [line_count] = vonalak_szama(e, no_sensors, dist_sensors)
line_count = 0;
    for i=1:no_sensors
            if (e(i)>0.6)  
                if (i==1)
                line_count = line_count +1;
                elseif e(i-1)<0.6
                    line_count = line_count +1;
                end
            end
    end
end