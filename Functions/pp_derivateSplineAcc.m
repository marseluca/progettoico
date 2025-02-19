function splineout = pp_derivateSplineVel(splinein)

    spline_dotdot = splinein;

    spline_dotdot.order = splinein.order-1;
    
    for i = 1:2
        spline_dotdot.coefs(:,i) = splinein.coefs(:,i)*(splinein.order-i);
    end

    splineout = spline_dotdot;
    
end

