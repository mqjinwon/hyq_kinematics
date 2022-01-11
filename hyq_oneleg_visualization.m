%% basic setting, leg leg testing
clc
clear
syms joint1 joint2 joint3;

robot = hyq;
leg_idx = uint8(Leg.RH);

[t01, t12, t23, t34] = robot.legTransformation(leg_idx, joint1, joint2, joint3);
t04 = t01 * t12 * t23 * t34;
ee = t04(1:3, 4); % end-effector position

syms x y z;

syms t;
x_cubic = CubicHermiteSpline(-0.37, -0.5, 0,0,t,1);
z_cubic = CubicHermiteSpline(-0.58, -0.58, 1/2,-1/2,t,1);

for t = 0:0.1:1

    x = subs(x_cubic);
    y = -0.29;
    z = subs(z_cubic);

    [joint1, joint2, joint3] = robot.IK(leg_idx, x, y, z);
    
    test = {subs(t01), subs(t12), subs(t23), subs(t34)};
    
    x = [0]; y = [0]; z = [0];
    
    T = robot.transformation(eye(3), [0; 0; 0]);
    
    for i = 1:length(test)
        T = T * test{i};

        result = T(1:3,4);
        x = [x result(1)];
        y = [y result(2)];
        z = [z result(3)];
    end
    
    hold on
    
    p = plot3(x,y,z,'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
    p.LineWidth = 3;
    plot3(subs(x_cubic), -0.29, subs(z_cubic), '-o', 'Color', 'r', 'MarkerSize', 3, 'MarkerFaceColor','#FF0000');
    
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 1]);
    ax = gca;
    ax.XAxis.Color = 'r';
    ax.YAxis.Color = 'g';
    ax.ZAxis.Color = 'b';
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    
    view(-45,45)
    pause(0.01)
    delete(p)
end
