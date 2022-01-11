clear
clc
%% Test one case
syms x0 x1 x0_dot x1_dot t T real; % for cubic hermite spline
syms a b real; % for end-effector variable
syms delJoint1 delJoint2 delJoint3 real; % for joint moving

robot = hyq;
legIdx = uint8(Leg.RH);
dim = 1; % x, y, z
side = 0; % x0, x1

equation = ObstacleDerivativeEquation(robot, legIdx, dim, side);

% leg_idx: 3 axis: 1 a: -0.167942 b: -0.560336
% x0: -1.18495 x1: -0.29 v0: 0 v1: 0 t: 0.0846914 T: 0.104938
% deljoint1: 0 deljoint2: 0 deljoint3: 0
% value0.0164207

delJoint1 = 0;
delJoint2 = 0;
delJoint3 = 0;
a = -0.167942;
b = -0.560336;
x0 = -1.18495;
x1 = -0.29;
x0_dot = 0;
x1_dot = 0;
T = 0.104938;
t = 0.0846914;

value = ObstacleDerivativeValue(uint8(legIdx), uint8(dim), uint8(side), ...
                                                    delJoint1, delJoint2, delJoint3, ...
                                                    a, b, x0, x1, x0_dot, x1_dot, t, T);
fprintf("eqaution: %d, value: %d \n", double(subs(equation)), value);

%% testing many cases
for legIdx=0:3 % LF, LH, RF, RH
    for dim=0:2 % x, y, z
        for side=0:1 % x0, x1
            equation = ObstacleDerivativeEquation(robot, legIdx, dim, side);
            for delJoint1 = 0: pi/3: pi
            %     delJoint1 = 0;
                delJoint2 = 0;
                delJoint3 = 0;
                a = -0.37;
                b = -0.29;
                x0 = 0;
                x1 = 0;
                x0_dot = 1/2;
                x1_dot = -1/2;
                T = 1;
                t = 0;
                for t= 0 : T/3 : T
                    value = ObstacleDerivativeValue(uint8(legIdx), uint8(dim), uint8(side), ...
                                                    delJoint1, delJoint2, delJoint3, ...
                                                    a, b, x0, x1, x0_dot, x1_dot, t, T);
                    if double(subs(equation)) - value > 1e-5
                        fprintf("false \n");
                    end
                    fprintf("eqaution: %d, value: %d \n", double(subs(equation)), value);
                end
            end
        end
    end
end

