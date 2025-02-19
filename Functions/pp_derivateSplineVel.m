function splineout = pp_derivateSplineVel(splinein)

    spline_dot = splinein;

    spline_dot.order = splinein.order-1;

    for i = 1:3
        spline_dot.coefs(:,i) = splinein.coefs(:,i)*(splinein.order-i);
    end

    splineout = spline_dot;

end

