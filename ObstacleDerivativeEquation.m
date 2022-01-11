function jac = ObstacleDerivativeEquation(robot, legIdx, dim, side)
% 
% 
syms x0 x1 x0_dot x1_dot t T real; % for cubic hermite spline
syms a b real; % for end-effector variable
syms delJoint1 delJoint2 delJoint3 real; % for joint moving

poly = CubicHermiteSpline(x0, x1, x0_dot, x1_dot, t, T);

switch dim
    case 0
        [joint1, joint2, joint3] = robot.IK(legIdx, poly, a, b);
    case 1
        [joint1, joint2, joint3] = robot.IK(legIdx, a, poly, b);
    case 2
        [joint1, joint2, joint3] = robot.IK(legIdx, a, b, poly);
end

% add deljoint & forward kinematics
[t01, t12, t23, t34] = robot.legTransformation(legIdx, ...
                                                joint1 + delJoint1, ...
                                                joint2 + delJoint2, ...
                                                joint3 + delJoint3);

t04 = t01 * t12 * t23 * t34; % transfromation
ee = t04(1:3, 4); % end-effector position

% derivative(expect poly because to decrease equation complexity)
cost = ee(dim+1) - poly;

% which part do we derivative
if side == 0
    jac = diff(cost, x0);
elseif side == 1
    jac = diff(cost, x1);
end

end

