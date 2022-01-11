function y = CubicHermiteSpline(x0, x1, x0_dot, x1_dot, t, dt)
% CUBIC_HERMITE_SPLINE 이 함수의 요약 설명 위치
% cubic hermite spline
% x0: start position
% x1: end position
% x0_dot: start slope
% x1_dot: end slope
% t: time[0, dt]
% dt: total duration

a0 = x0;
a1 = x0_dot;
a2 = -dt^-2 *(3*(x0-x1) + dt*(2*x0_dot + x1_dot));
a3 = dt^-3 *(2*(x0-x1) + dt*(x0_dot + x1_dot));

y = a0 + a1*t + a2*t^2 + a3*t^3;
end

