clear
clc

syms x0 x1 x0_dot x1_dot t T real; % for cubic hermite spline
syms a b real; % for end-effector variable
syms delJoint1 delJoint2 delJoint3 real; % for joint moving

robot = hyq;
legIdx = uint8(Leg.LF);
dim = 2; % x, y, z
side = 1; % x0, x1

%% testing many cases
for legIdx=0:3 % LF, LH, RF, RH
    for dim=0:2 % x, y, z
        for side=0:1 % x0, x1
            equation = ObstacleDerivativeEquation(robot, legIdx, dim, side);
            for delJoint1 = 0: pi/10: pi
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
                for t= 0 : 0.1 : T
                    value = ObstacleDerivativeValue(legIdx, dim, side, ...
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

