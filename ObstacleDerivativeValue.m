function output = ObstacleDerivativeValue(legIdx, dim, side, ...
                                        delJoint1, delJoint2, delJoint3, ...
                                        a, b, x0, x1, x0_dot, x1_dot, t, T)

output = 0;
s1=0;s2=0;s3=0;s4=0;s5=0;s6=0;s7=0;s8=0;s9=0;s10=0;s11=0;s12=0;s13=0;s14=0;s15=0;s16=0;s17=0;s18=0;s19=0;

b2 = b^2;
t2 = t^2;
t3 = t^3;
T2 = T^2;
T3 = T^3;

if dim == 0 % x
    if legIdx == 0 % x, LF
        s10 = sqrt((a-207/1000)^2 + b2);
        s9 = t3*(2*x0-2*x1 + T*(x0_dot + x1_dot));
        s8 = t2*(3*x0-3*x1+T*(2*x0_dot+x1_dot));
        s7 = (a-207/1000)^2/s10 + b2/s10 - 41/500;
        s6 = x0+t*x0_dot - s8 + s9 - 747/2000;

        if side == 0 % x, LF, start
            s5 = 2*t3/T3 - 3*t2/T2 + 1;

        elseif side == 1 % x, LF, end
            s5 = 3*t2/T2 - 2*t3/T^4; % LF diff
        end

        s4 = delJoint3 + acos(complex(1-200*s6^2/49-200*s7^2/49));
        s3 = 7*sqrt(complex(1-(200*s7^2/49 + 200*s6^2/49 - 1)^2));
        s2 = delJoint2 - pi/2 + angle(x0 + (a-207/1000)^2*1i/s10 + t*x0_dot + b2*1i/s10 - s8 + s9 - 747/2000 - 41*1i/500)+ acos(complex(10*sqrt(s7^2+s6^2)/7));
        s1 = s5*s7/(s7^2+s6^2) + 10*s5*s6/(7*sqrt(s7^2+s6^2)*sqrt(complex(1-100*s6^2/49-100*s7^2/49)));
        output = 7*cos(s2)*s1/20 - 7*cos(s4)*cos(s2)*s1/20 + 7*sin(s4)*sin(s2)*s1/20 + 20*cos(s4)*cos(s2)*s5*s6/s3 - 20*sin(s4)*sin(s2)*s5*s6/s3;

    elseif legIdx == 1 % x, LH
        s10 = sqrt((a-207/1000)^2 + b2);
        s9 = t3*(2*x0-2*x1 + T*(x0_dot + x1_dot));
        s8 = t2*(3*x0-3*x1+T*(2*x0_dot+x1_dot));
        s7 = (a-207/1000)^2/s10 + b2/s10 - 41/500;
        s6 = x0+t*x0_dot - s8 + s9 + 747/2000; % LF diff

        if side == 0 % x, LH, start
            s5 = 2*t3/T3 - 3*t2/T2 + 1; % LF same
        elseif side == 1 % x, LH, end
            s5 = 3*t2/T2 - 2*t3/T^4; % LF same
        end

        s4 = delJoint3 - acos(complex(1-200*s6^2/49-200*s7^2/49)); % LF diff
        s3 = 7*sqrt(complex(1-(200*s7^2/49 + 200*s6^2/49 - 1)^2)); % LF same
        s2 = delJoint2 + pi/2 - angle(-x0 + (a-207/1000)^2*1i/s10 - t*x0_dot + b2*1i/s10 + s8 - s9 - 747/2000 - 41*1i/500)- acos(complex(10*sqrt(s7^2+s6^2)/7)); % LF diff
        s1 = s5*s7/(s7^2+s6^2) - 10*s5*s6/(7*sqrt(s7^2+s6^2)*sqrt(complex(1-100*s6^2/49-100*s7^2/49))); % LF diff
        output = 7*cos(s2)*s1/20  + 7*sin(s4)*sin(s2)*s1/20 - 7*cos(s4)*cos(s2)*s1/20 - 20*cos(s4)*cos(s2)*s5*s6/s3 + 20*sin(s4)*sin(s2)*s5*s6/s3; % LF diff

    elseif legIdx == 2 % x, RF
        s10 = sqrt((a+207/1000)^2 + b2);
        s9 = t3*(2*x0-2*x1 + T*(x0_dot + x1_dot));
        s8 = t2*(3*x0-3*x1+T*(2*x0_dot+x1_dot));
        s7 = (a+207/1000)^2/s10 + b2/s10 - 41/500;
        s6 = x0+t*x0_dot - s8 + s9 - 747/2000; % LF diff

        if side == 0 % x, RH, start
            s5 = 2*t3/T3 - 3*t2/T2 + 1; % LF same
        elseif side == 1 % x, RH, end
            s5 = 3*t2/T2 - 2*t3/T^4; % LF same
        end

        s4 = delJoint3 + acos(complex(1-200*s6^2/49-200*s7^2/49)); % LF diff
        s3 = 7*sqrt(complex(1-(200*s7^2/49 + 200*s6^2/49 - 1)^2)); % LF same
        s2 = delJoint2 - pi/2 + angle(x0 + (a+207/1000)^2*1i/s10 + t*x0_dot + b2*1i/s10 - s8 + s9 - 747/2000 - 41*1i/500) + acos(complex(10*sqrt(s7^2+s6^2)/7)); % LF diff
        s1 = s5*s7/(s7^2+s6^2) + 10*s5*s6/(7*sqrt(s7^2+s6^2)*sqrt(complex(1-100*s6^2/49-100*s7^2/49))); % LF diff
        output = 7*cos(s2)*s1/20 - 7*cos(s4)*cos(s2)*s1/20 + 7*sin(s4)*sin(s2)*s1/20  + 20*cos(s4)*cos(s2)*s5*s6/s3 - 20*sin(s4)*sin(s2)*s5*s6/s3; % LF diff

    elseif legIdx == 3 % x, RH
        s10 = sqrt((a+207/1000)^2 + b2);
        s9 = t3*(2*x0-2*x1 + T*(x0_dot + x1_dot));
        s8 = t2*(3*x0-3*x1+T*(2*x0_dot+x1_dot));
        s7 = (a+207/1000)^2/s10 + b2/s10 - 41/500;
        s6 = x0+t*x0_dot - s8 + s9 + 747/2000; % LF diff

        if side == 0 % x, RH, start
            s5 = 2*t3/T3 - 3*t2/T2 + 1; % LF same
        elseif side == 1 % x, RH, end
            s5 = 3*t2/T2 - 2*t3/T^4; % LF same
        end

        s4 = delJoint3 - acos(complex(1-200*s6^2/49-200*s7^2/49)); % LF diff
        s3 = 7*sqrt(complex(1-(200*s7^2/49 + 200*s6^2/49 - 1)^2)); % LF same
        s2 = delJoint2 + pi/2 - angle(-x0 + (a+207/1000)^2*1i/s10 - t*x0_dot + b2*1i/s10 + s8 - s9 - 747/2000 - 41*1i/500)- acos(complex(10*sqrt(s7^2+s6^2)/7)); % LF diff
        s1 = s5*s7/(s7^2+s6^2) - 10*s5*s6/(7*sqrt(s7^2+s6^2)*sqrt(complex(1-100*s6^2/49-100*s7^2/49))); % LF diff
        output = 7*cos(s2)*s1/20  + 7*sin(s4)*sin(s2)*s1/20 - 7*cos(s4)*cos(s2)*s1/20 - 20*cos(s4)*cos(s2)*s5*s6/s3 + 20*sin(s4)*sin(s2)*s5*s6/s3; % LF diff


    end
elseif dim == 1 % y

    if legIdx == 0 % y, LF
        s17 = 2*x0 - 2*x1 + T*(x0_dot + x1_dot);
        s16 = 3*x0 - 3*x1 + T*(2*x0_dot + x1_dot);
        s15 = b - x0*1i - t*x0_dot*1i + t2*s16*1i/T2 - t3*s17*1i/T3 + 207/1000 * 1i;
        s14 = x0 + t*x0_dot - t2*s16/T2 + t3*s17/T3 - 207/1000;
        s13 = b2/abs(s15) + s14^2/abs(s15) - 41/500;
        s12 = (a-747/2000)^2;

        if side == 0 % y, LF, start
            s11 = -3*t2*1i/T2 + 2*t3*1i/T3 + 1i;
            s10 = 2*t3/T3 -3*t2/T2 + 1;
        elseif side == 1 % y, LF, end
            s11 = 3*t2*1i/T2 - 2*t3*1i/T3;
            s10 = 3*t2/T2 -2*t3/T3;
        end

        s9 = 200*s13^2/49;
        s8 = sqrt(s12 + s13^2);
        s7 = 2*s10*s14/abs(s15) + b2*sign(s15)*s11/abs(s15)^2 + sign(s15)*s11*s14^2/abs(s15)^2;
        s6 = b2 + s14^2;
        s5 = delJoint1 -atan2(s14, -b);
        s4 = delJoint3 + acos(complex(1-s9-200*s12/49));
        s3 = 7*sqrt(complex(1-(200*s12/49+s9-1)^2));
        s2 = delJoint2 -pi/2 + angle(a+b2*1i/abs(s15) + s14^2*1i/abs(s15) - 747/2000 - 41/500*1i) + acos(complex(10*s8/7));
        s1 = (a-747/2000)*s7/(s12+s13^2)-10*s7*s13/(7*s8*sqrt(complex(1-100*s13^2/49-100*s12/49)));

        output = 7*sin(s5)*sin(s2)*s1/20 - 7*sin(s5)*cos(s4)*sin(s2)*s1/20 -7*sin(s5)*cos(s2)*sin(s4)*s1/20 ...
                -41*b*cos(s5)*s10/(500*s6) ...
                -7*b*cos(s5)*cos(s2)*s10/(20*s6) + 7*b*cos(s5)*cos(s4)*cos(s2)*s10/(20*s6) -7*b*cos(s5)*sin(s4)*sin(s2)*s10/(20*s6)...
                -20*sin(s5)*cos(s4)*sin(s2)*s7*s13/s3 -20*sin(s5)*cos(s2)*sin(s4)*s7*s13/s3;
    elseif legIdx == 1 % y, LH
        s17 = 2*x0 - 2*x1 + T*(x0_dot + x1_dot);
        s16 = 3*x0 - 3*x1 + T*(2*x0_dot + x1_dot);
        s15 = b - x0*1i - t*x0_dot*1i + t2*s16*1i/T2 - t3*s17*1i/T3 + 207/1000 * 1i;
        s14 = x0 + t*x0_dot - t2*s16/T2 + t3*s17/T3 - 207/1000;
        s13 = b2/abs(s15) + s14^2/abs(s15) - 41/500;
        s12 = (a+747/2000)^2;

        if side == 0 % y, LH, start
            s11 = -3*t2*1i/T2 + 2*t3*1i/T3 + 1i;
            s10 = 2*t3/T3 -3*t2/T2 + 1;
        elseif side == 1 % y, LH, end
            s11 = 3*t2*1i/T2 - 2*t3*1i/T3;
            s10 = 3*t2/T2 -2*t3/T3;
        end

        s9 = 200*s13^2/49;
        s8 = sqrt(s12 + s13^2);
        s7 = 2*s10*s14/abs(s15) + b2*sign(s15)*s11/abs(s15)^2 + sign(s15)*s11*s14^2/abs(s15)^2;
        s6 = b2 + s14^2;
        s5 = delJoint1 -atan2(s14, -b);
        s4 = delJoint3 - acos(complex(1-s9-200*s12/49));
        s3 = 7*sqrt(complex(1-(200*s12/49+s9-1)^2));
        s2 = delJoint2 +pi/2 - acos(complex(10*s8/7)) - angle(-a+b2*1i/abs(s15) + s14^2*1i/abs(s15) - 747/2000 - 41/500*1i);
        s1 = (a+747/2000)*s7/(s12+s13^2) + 10*s7*s13/(7*s8*sqrt(complex(1-100*s13^2/49-100*s12/49)));

        output = 7*sin(s5)*sin(s2)*s1/20 - 7*sin(s5)*cos(s4)*sin(s2)*s1/20 -7*sin(s5)*cos(s2)*sin(s4)*s1/20 ...
                -41*b*cos(s5)*s10/(500*s6) ...
                -7*b*cos(s5)*cos(s2)*s10/(20*s6) + 7*b*cos(s5)*cos(s4)*cos(s2)*s10/(20*s6) -7*b*cos(s5)*sin(s4)*sin(s2)*s10/(20*s6)...
                +20*sin(s5)*cos(s4)*sin(s2)*s7*s13/s3 +20*sin(s5)*cos(s2)*sin(s4)*s7*s13/s3;
    elseif legIdx == 2 % y, RF
        s19 = 2*x0 - 2*x1 + T*(x0_dot + x1_dot);
        s18 = 3*x0 - 3*x1 + T*(2*x0_dot + x1_dot);
        s17 = t3*s19/T3;
        s16 = t2*s18/T2;
        s15 = x0 + t*x0_dot - s16 + s17 + 207/1000;
        s14 = b + x0*1i + t*x0_dot*1i - t2*s18*1i/T2 + t3*s19*1i/T3 + 207/1000 * 1i;
        s13 = b2/abs(s14) + s15^2/abs(s14) - 41/500;
        s12 = (a-747/2000)^2;

        if side == 0 % y, RF, start
            s11 = -3*t2*1i/T2 + 2*t3*1i/T3 + 1i;
            s10 = 2*t3/T3 -3*t2/T2 + 1;
        elseif side == 1 % y, RF, end
            s11 = 3*t2*1i/T2 - 2*t3*1i/T3;
            s10 = 3*t2/T2 -2*t3/T3;
        end

        s9 = 200*s13^2/49;
        s8 = sqrt(s12 + s13^2);
        s7 = -2*s10*s15/abs(s14) + b2*sign(s14)*s11/abs(s14)^2 + sign(s14)*s11*s15^2/abs(s14)^2;
        s6 = b2 + s15^2;
        s5 = delJoint3 + acos(complex(1-s9-200*s12/49));
        s4 = 7*sqrt(complex(1-(200*s12/49+s9-1)^2));
        s3 = delJoint1 -atan2(s16 -t*x0_dot - x0 - s17 - 207/1000, -b);
        s2 = delJoint2 -pi/2 + angle(a+b2*1i/abs(s14) + s15^2*1i/abs(s14) - 747/2000 - 41/500*1i) + acos(complex(10*s8/7));
        s1 = (a-747/2000)*s7/(s12+s13^2)-10*s7*s13/(7*s8*sqrt(complex(1-100*s13^2/49-100*s12/49)));

        output = 7*sin(s3)*sin(s2)*s1/20 - 7*sin(s3)*cos(s5)*sin(s2)*s1/20 -7*sin(s3)*cos(s2)*sin(s5)*s1/20 ...
                -41*b*cos(s3)*s10/(500*s6) ...
                -7*b*cos(s3)*cos(s2)*s10/(20*s6) + 7*b*cos(s3)*cos(s5)*cos(s2)*s10/(20*s6) -7*b*cos(s3)*sin(s5)*sin(s2)*s10/(20*s6)...
                -20*sin(s3)*cos(s5)*sin(s2)*s7*s13/s4 -20*sin(s3)*cos(s2)*sin(s5)*s7*s13/s4;
    elseif legIdx == 3 % y, RH
        s19 = 2*x0 - 2*x1 + T*(x0_dot + x1_dot);
        s18 = 3*x0 - 3*x1 + T*(2*x0_dot + x1_dot);
        s17 = t3*s19/T3;
        s16 = t2*s18/T2;
        s15 = x0 + t*x0_dot - s16 + s17 + 207/1000;
        s14 = b + x0*1i + t*x0_dot*1i - t2*s18*1i/T2 + t3*s19*1i/T3 + 207/1000 * 1i;
        s13 = b2/abs(s14) + s15^2/abs(s14) - 41/500;
        s12 = (a+747/2000)^2;

        if side == 0 % y, RH, start
            s11 = -3*t2*1i/T2 + 2*t3*1i/T3 + 1i;
            s10 = 2*t3/T3 -3*t2/T2 + 1;
        elseif side == 1 % y, RH, end
            s11 = 3*t2*1i/T2 - 2*t3*1i/T3;
            s10 = 3*t2/T2 -2*t3/T3;
        end

        s9 = 200*s13^2/49;
        s8 = sqrt(s12 + s13^2);
        s7 = -2*s10*s15/abs(s14) + b2*sign(s14)*s11/abs(s14)^2 + sign(s14)*s11*s15^2/abs(s14)^2;
        s6 = b2 + s15^2;
        s5 = delJoint3 - acos(complex(1-s9-200*s12/49));
        s4 = 7*sqrt(complex(1-(200*s12/49+s9-1)^2));
        s3 = delJoint1 -atan2(s16 -t*x0_dot - x0 - s17 - 207/1000, -b);
        s2 = delJoint2 +pi/2 - acos(complex(10*s8/7)) - angle(-a+b2*1i/abs(s14) + s15^2*1i/abs(s14) - 747/2000 - 41/500*1i);
        s1 = (a+747/2000)*s7/(s12+s13^2) + 10*s7*s13/(7*s8*sqrt(complex(1-100*s13^2/49-100*s12/49)));

        output = 7*sin(s3)*sin(s2)*s1/20 - 7*sin(s3)*cos(s5)*sin(s2)*s1/20 -7*sin(s3)*cos(s2)*sin(s5)*s1/20 ...
                -41*b*cos(s3)*s10/(500*s6) ...
                -7*b*cos(s3)*cos(s2)*s10/(20*s6) + 7*b*cos(s3)*cos(s5)*cos(s2)*s10/(20*s6) -7*b*cos(s3)*sin(s5)*sin(s2)*s10/(20*s6)...
                +20*sin(s3)*cos(s5)*sin(s2)*s7*s13/s4 +20*sin(s3)*cos(s2)*sin(s5)*s7*s13/s4;
    end
elseif dim == 2 % z

    if legIdx == 0 % z, LF
        s17 = t3 * (2*x0 - 2*x1 + T * (x0_dot + x1_dot))/T3;
        s16 = t2 * (3*x0 - 3*x1 + T * (2*x0_dot + x1_dot))/T2;
        s15 = x0 + t*x0_dot - s16 + s17;
        s14 = x0 - b*1i + t*x0_dot - s16 + s17 + 207/1000 * 1i;
        s13 = (b-207/1000)^2;
        s12 = s15^2/abs(s14) + s13/abs(s14) - 41/500;
        s11 = (a-747/2000)^2;
        if side == 0 % z, LF, start
            s10 = 2*t3/T3 - 3*t2/T2 + 1;
        elseif side == 1 % z, LF, end
            s10 = 3*t2/T2 - 2*t3/T3;
        end
        s9 = 200*s12^2/49;
        s8 = sqrt(s11 + s12^2);
        s7 = -2*s10*s15/abs(s14) + sign(s14)*s13*s10/abs(s14)^2 + sign(s14)*s10*s15^2/abs(s14)^2;
        s6 = 20 * (s13 + s15^2);
        s5 = delJoint3 + acos(complex(1 - s9 - 200*s11/49));
        s4 = 7 * sqrt(complex(1-(200*s11/49 + s9 - 1)^2));
        s3 = delJoint1 -atan2(b-207/1000, s16 - t*x0_dot - x0 - s17);
        s2 = delJoint2 -pi/2 + angle(a + s15^2 * 1i/abs(s14) + s13 * 1i/abs(s14) - 747/2000 - 41/500*1i) + acos(complex(10*s8/7));
        s1 = (a-747/2000)*s7/(s11+s12^2)-10*s7*s12/(7*s8*sqrt(complex(1-100*s12^2/49-100*s11/49)));
        output = -7*cos(s3)*sin(s2)*s1/20 - 41*sin(s3)*(b-207/1000)*s10/(500*(s13+s15^2)) ...
                + 7*cos(s5)*cos(s3)*sin(s2)*s1/20 + 7*sin(s5)*cos(s3)*cos(s2)*s1/20 ...
                - 7*cos(s2)*sin(s3)*(b-207/1000)*s10/s6 ...
                + 20*cos(s5)*cos(s3)*sin(s2)*s7*s12/s4 + 20*sin(s5)*cos(s3)*cos(s2)*s7*s12/s4 ...
                + 7 * cos(s5)*cos(s2)*sin(s3)*(b-207/1000)*s10/s6 - 7*sin(s5)*sin(s3)*sin(s2)*(b-207/1000)*s10/s6;

    elseif legIdx == 1 % z, LH
        s17 = t3 * (2*x0 - 2*x1 + T * (x0_dot + x1_dot))/T3;
        s16 = t2 * (3*x0 - 3*x1 + T * (2*x0_dot + x1_dot))/T2;
        s15 = x0 + t*x0_dot - s16 + s17;
        s14 = x0 - b*1i + t*x0_dot - s16 + s17 + 207/1000 * 1i;
        s13 = (b-207/1000)^2;
        s12 = s15^2/abs(s14) + s13/abs(s14) - 41/500;
        s11 = (a+747/2000)^2;
        if side == 0 % z, LH, start
            s10 = 2*t3/T3 - 3*t2/T2 + 1;
        elseif side == 1 % z, LH, end
            s10 = 3*t2/T2 - 2*t3/T3;
        end
        s9 = 200*s12^2/49;
        s8 = sqrt(s11 + s12^2);
        s7 = -2*s10*s15/abs(s14) + sign(s14)*s13*s10/abs(s14)^2 + sign(s14)*s10*s15^2/abs(s14)^2;
        s6 = 20 * (s13 + s15^2);
        s5 = delJoint3 - acos(complex(1 - s9 - 200*s11/49));
        s4 = 7 * sqrt(complex(1-(200*s11/49 + s9 - 1)^2));
        s3 = delJoint1 -atan2(b-207/1000, s16 - t*x0_dot - x0 - s17);
        s2 = delJoint2 +pi/2 - angle(-a + s15^2 * 1i/abs(s14) + s13 * 1i/abs(s14) - 747/2000 - 41/500*1i) - acos(complex(10*s8/7));
        s1 = (a+747/2000)*s7/(s11+s12^2)+10*s7*s12/(7*s8*sqrt(complex(1-100*s12^2/49-100*s11/49)));
        output = -7*cos(s3)*sin(s2)*s1/20 - 41*sin(s3)*(b-207/1000)*s10/(500*(s13+s15^2)) ...
                + 7*cos(s5)*cos(s3)*sin(s2)*s1/20 + 7*sin(s5)*cos(s3)*cos(s2)*s1/20 ...
                - 7*cos(s2)*sin(s3)*(b-207/1000)*s10/s6 ...
                - 20*cos(s5)*cos(s3)*sin(s2)*s7*s12/s4 - 20*sin(s5)*cos(s3)*cos(s2)*s7*s12/s4 ...
                + 7 * cos(s5)*cos(s2)*sin(s3)*(b-207/1000)*s10/s6 - 7*sin(s5)*sin(s3)*sin(s2)*(b-207/1000)*s10/s6;

    elseif legIdx == 2 % z, RF
        s17 = t3 * (2*x0 - 2*x1 + T * (x0_dot + x1_dot))/T3;
        s16 = t2 * (3*x0 - 3*x1 + T * (2*x0_dot + x1_dot))/T2;
        s15 = x0 + t*x0_dot - s16 + s17;
        s14 = x0 + b*1i + t*x0_dot - s16 + s17 + 207/1000 * 1i;
        s13 = (b+207/1000)^2;
        s12 = s15^2/abs(s14) + s13/abs(s14) - 41/500;
        s11 = (a-747/2000)^2;
        if side == 0 % z, RF, start
            s10 = 2*t3/T3 - 3*t2/T2 + 1;
        elseif side == 1 % z, RF, end
            s10 = 3*t2/T2 - 2*t3/T3;
        end
        s9 = 200*s12^2/49;
        s8 = sqrt(s11 + s12^2);
        s7 = -2*s10*s15/abs(s14) + sign(s14)*s13*s10/abs(s14)^2 + sign(s14)*s10*s15^2/abs(s14)^2;
        s6 = 20 * (s13 + s15^2);
        s5 = delJoint3 + acos(complex(1 - s9 - 200*s11/49));
        s4 = 7 * sqrt(complex(1-(200*s11/49 + s9 - 1)^2));
        s3 = delJoint1 -atan2(-b-207/1000, s16 - t*x0_dot - x0 - s17);
        s2 = delJoint2 -pi/2 + angle(a + s15^2 * 1i/abs(s14) + s13 * 1i/abs(s14) - 747/2000 - 41/500*1i) + acos(complex(10*s8/7));
        s1 = (a-747/2000)*s7/(s11+s12^2) - 10*s7*s12/(7*s8*sqrt(complex(1-100*s12^2/49-100*s11/49)));
        output = -7*cos(s3)*sin(s2)*s1/20 + 41*sin(s3)*(b+207/1000)*s10/(500*(s13+s15^2)) ...
                + 7*cos(s5)*cos(s3)*sin(s2)*s1/20 + 7*sin(s5)*cos(s3)*cos(s2)*s1/20 ...
                + 7*cos(s2)*sin(s3)*(b+207/1000)*s10/s6 ...
                + 20*cos(s5)*cos(s3)*sin(s2)*s7*s12/s4 + 20*sin(s5)*cos(s3)*cos(s2)*s7*s12/s4 ...
                - 7 * cos(s5)*cos(s2)*sin(s3)*(b+207/1000)*s10/s6 + 7*sin(s5)*sin(s3)*sin(s2)*(b+207/1000)*s10/s6;

    elseif legIdx == 3 % z, RH
        s17 = t3 * (2*x0 - 2*x1 + T * (x0_dot + x1_dot))/T3;
        s16 = t2 * (3*x0 - 3*x1 + T * (2*x0_dot + x1_dot))/T2;
        s15 = x0 + t*x0_dot - s16 + s17;
        s14 = x0 + b*1i + t*x0_dot - s16 + s17 + 207/1000 * 1i;
        s13 = (b+207/1000)^2;
        s12 = s15^2/abs(s14) + s13/abs(s14) - 41/500;
        s11 = (a+747/2000)^2;
        if side == 0 % z, RH, start
            s10 = 2*t3/T3 - 3*t2/T2 + 1;
        elseif side == 1 % z, RH, end
            s10 = 3*t2/T2 - 2*t3/T3;
        end
        s9 = 200*s12^2/49;
        s8 = sqrt(s11 + s12^2);
        s7 = -2*s10*s15/abs(s14) + sign(s14)*s13*s10/abs(s14)^2 + sign(s14)*s10*s15^2/abs(s14)^2;
        s6 = 20 * (s13 + s15^2);
        s5 = delJoint3 - acos(complex(1 - s9 - 200*s11/49));
        s4 = 7 * sqrt(complex(1-(200*s11/49 + s9 - 1)^2));
        s3 = delJoint1 -atan2(-b-207/1000, s16 - t*x0_dot - x0 - s17);
        s2 = delJoint2 +pi/2 - angle(-a + s15^2 * 1i/abs(s14) + s13 * 1i/abs(s14) - 747/2000 - 41/500*1i) - acos(complex(10*s8/7));
        s1 = (a+747/2000)*s7/(s11+s12^2) + 10*s7*s12/(7*s8*sqrt(complex(1-100*s12^2/49-100*s11/49)));

        output = -7*cos(s3)*sin(s2)*s1/20 + 41*sin(s3)*(b+207/1000)*s10/(500*(s13+s15^2)) ...
                + 7*cos(s5)*cos(s3)*sin(s2)*s1/20 + 7*sin(s5)*cos(s3)*cos(s2)*s1/20 ...
                + 7*cos(s2)*sin(s3)*(b+207/1000)*s10/s6 ...
                - 20*cos(s5)*cos(s3)*sin(s2)*s7*s12/s4 - 20*sin(s5)*cos(s3)*cos(s2)*s7*s12/s4 ...
                - 7 * cos(s5)*cos(s2)*sin(s3)*(b+207/1000)*s10/s6 + 7*sin(s5)*sin(s3)*sin(s2)*(b+207/1000)*s10/s6;
    end
end

poly=0;

if side == 0
    poly = 2*t3/T3 - 3*t2/T2 + 1;
elseif side == 1
    poly = 3*t2/T2 - 2*t3/T3;
end

output = real(output-poly);

end

