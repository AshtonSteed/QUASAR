 function [p, v, a, yaw_des] = trajectory_gen(t, shape_id, width, height, speed, t_total)

    p = zeros(3,1); v = zeros(3,1); a = zeros(3,1); yaw_des = 0;

    

    if t > t_total, t = t_total; end

    

    switch shape_id

        case 1 % HELIX

            radius = width / 2;

            omega = speed; 

            v_z = height / t_total;

            p = [radius * cos(omega * t); radius * sin(omega * t); v_z * t];

            v = [-radius * omega * sin(omega * t); radius * omega * cos(omega * t); v_z];

            yaw_des = omega * t; 

                 

        case 2 % FIGURE-8

            A = width / 2; 

            wx = speed; wy = 2 * speed; 

            p = [A * sin(wx * t); A * sin(wy * t); height]; 

            v = [A * wx * cos(wx * t); A * wy * cos(wy * t); 0];

            yaw_des = atan2(v(2), v(1));

                 

        case 3 % SQUARE

            leg_time = t_total / 4;

            v_xy = width / leg_time; 

            p(3) = height; 

            if t <= leg_time

                p(1) = v_xy * t; v(1) = v_xy;

            elseif t <= 2*leg_time

                p(1) = width; p(2) = v_xy * (t - leg_time); v(2) = v_xy;

            elseif t <= 3*leg_time

                p(1) = width - (v_xy * (t - 2*leg_time)); p(2) = width; v(1) = -v_xy;

            else

                p(1) = 0; p(2) = width - (v_xy * (t - 3*leg_time)); v(2) = -v_xy;

            end

            yaw_des = atan2(v(2), v(1));

    end

end