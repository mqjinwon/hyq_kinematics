classdef hyq
    % hyq 이 클래스의 요약 설명 위치
    %   자세한 설명 위치
    
    properties
        LF_T, LF_R
        LH_T, LH_R
        RF_T, RF_R
        RH_T, RH_R

        L1, L2, L3
        baseTohip
    end
    
    methods

        function obj = hyq()
            obj.LF_T = {[0.3735; 0.207; 0;];
                        [0.082; 0; 0;];
                        [0.35; 0; 0;];
                        [0.35; 0; 0;];};
            obj.LF_R = {obj.rotZ(pi) * obj.rotY(pi/2) * obj.rotX(0);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(pi/2);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(0);
                        obj.rotZ(-pi/2) * obj.rotY(0) * obj.rotX(pi/2);};
            obj.LH_T = {[-0.3735; 0.207; 0;];
                        [0.082; 0; 0;];
                        [0.35; 0; 0;];
                        [0.35; 0; 0;];};
            obj.LH_R = {obj.rotZ(pi) * obj.rotY(pi/2) * obj.rotX(0);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(pi/2);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(0);
                        obj.rotZ(-pi/2) * obj.rotY(0) * obj.rotX(pi/2);};
            obj.RF_T = {[0.3735; -0.207; 0;];
                        [0.082; 0; 0;];
                        [0.35; 0; 0;];
                        [0.35; 0; 0;];};
            obj.RF_R = {obj.rotZ(0) * obj.rotY(pi/2) * obj.rotX(0);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(-pi/2);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(0);
                        obj.rotZ(-pi/2) * obj.rotY(0) * obj.rotX(pi/2);};
            obj.RH_T = {[-0.3735; -0.207; 0;];
                        [0.082; 0; 0;];
                        [0.35; 0; 0;];
                        [0.35; 0; 0;];};
            obj.RH_R = {obj.rotZ(0) * obj.rotY(pi/2) * obj.rotX(0);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(-pi/2);
                        obj.rotZ(0) * obj.rotY(0) * obj.rotX(0);
                        obj.rotZ(-pi/2) * obj.rotY(0) * obj.rotX(pi/2);};

            obj.L1 = 0.082;
            obj.L2 = 0.35;
            obj.L3 = 0.35;
            
            obj.baseTohip = [0.3735; 0.207; 0.0;];
        end

        function matrix = rotX(~, radian)
        %ROTX 이 함수의 요약 설명 위치
        %   x-axis를 기준으로 한 rotation matrix
        matrix = [1     0             0;
                  0 cos(radian)  -sin(radian);
                  0 sin(radian)  cos(radian);];
        end

        function matrix = rotY(~, radian)
        %ROTY 이 함수의 요약 설명 위치
        %  y-axis를 기준으로 한 rotation matrix
        matrix = [cos(radian) 0 sin(radian) ;
                  0             1       0;
                  -sin(radian) 0 cos(radian);];
         end
        
        function matrix = rotZ(~, radian)
        %ROTZ 이 함수의 요약 설명 위치
        %  z-axis를 기준으로 한 rotation matrix
        matrix = [cos(radian) -sin(radian) 0;
                  sin(radian) cos(radian)  0;
                  0              0         1;];
        end

        function matrix = transformation(~, rotation, translation)
        matrix = rotation;
        matrix(1 : 3, 4) = translation;
        matrix(4,4) = 1;
        end

        function matrix = rotationMatrix(obj, acuator, rot, tran)
        %ROTATION_MATRIX 이 함수의 요약 설명 위치
        % revolute joint = acuator
        onesMatrix = eye(3,3);
        zeroVec = zeros(3,1);
        matrix = obj.transformation(onesMatrix, tran);
        matrix = matrix * obj.transformation(rot, zeroVec);
        matrix = round(matrix, 5);
        matrix = matrix * obj.transformation(acuator, zeroVec);
        end

        function [t01, t12, t23, t34] = legTransformation(obj, legIdx, joint1, joint2, joint3)
            rotJoint1 = obj.rotZ(joint1);
            rotJoint2 = obj.rotZ(joint2);
            rotJoint3 = obj.rotZ(joint3);

            switch legIdx
                case 0
                    t01 = obj.rotationMatrix(rotJoint1, cell2mat(obj.LF_R(1)), cell2mat(obj.LF_T(1)));
                    t12 = obj.rotationMatrix(rotJoint2, cell2mat(obj.LF_R(2)), cell2mat(obj.LF_T(2)));
                    t23 = obj.rotationMatrix(rotJoint3, cell2mat(obj.LF_R(3)), cell2mat(obj.LF_T(3)));
                    t34 = obj.rotationMatrix(obj.rotX(0), cell2mat(obj.LF_R(4)), cell2mat(obj.LF_T(4)));
                case 1
                    t01 = obj.rotationMatrix(rotJoint1, cell2mat(obj.LH_R(1)), cell2mat(obj.LH_T(1)));
                    t12 = obj.rotationMatrix(rotJoint2, cell2mat(obj.LH_R(2)), cell2mat(obj.LH_T(2)));
                    t23 = obj.rotationMatrix(rotJoint3, cell2mat(obj.LH_R(3)), cell2mat(obj.LH_T(3)));
                    t34 = obj.rotationMatrix(obj.rotX(0), cell2mat(obj.LH_R(4)), cell2mat(obj.LH_T(4)));
                case 2
                    t01 = obj.rotationMatrix(rotJoint1, cell2mat(obj.RF_R(1)), cell2mat(obj.RF_T(1)));
                    t12 = obj.rotationMatrix(rotJoint2, cell2mat(obj.RF_R(2)), cell2mat(obj.RF_T(2)));
                    t23 = obj.rotationMatrix(rotJoint3, cell2mat(obj.RF_R(3)), cell2mat(obj.RF_T(3)));
                    t34 = obj.rotationMatrix(obj.rotX(0), cell2mat(obj.RF_R(4)), cell2mat(obj.RF_T(4)));
                case 3
                    t01 = obj.rotationMatrix(rotJoint1, cell2mat(obj.RH_R(1)), cell2mat(obj.RH_T(1)));
                    t12 = obj.rotationMatrix(rotJoint2, cell2mat(obj.RH_R(2)), cell2mat(obj.RH_T(2)));
                    t23 = obj.rotationMatrix(rotJoint3, cell2mat(obj.RH_R(3)), cell2mat(obj.RH_T(3)));
                    t34 = obj.rotationMatrix(obj.rotX(0), cell2mat(obj.RH_R(4)), cell2mat(obj.RH_T(4)));
            end
        end
        
        function [joint1, joint2, joint3] = IK(obj, legIdx, x, y, z)
            % HYQIK 이 함수의 요약 설명 위치
            % legIdx: LF, LH, RF, RH
            
            xr = [x; y; z;];
            
            switch(legIdx)
                case 0
                    xr = xr .* [1; 1; 1;];
                case 1
                    xr = xr .* [-1; 1; 1;];
                case 2
                    xr = xr .* [1; -1; 1;];
                case 3
                    xr = xr .* [-1; -1; 1;];
                otherwise
            end
            
            xr = xr - obj.baseTohip;
            
            q_HAA_bf = -atan2(xr(2), -xr(3));
            % q_HAA_bf = atan(xr(2)/xr(3)); % changed
            q_HAA_br = q_HAA_bf;
            
            R = obj.rotX(q_HAA_bf);
            
            xr = R * xr;
            
            hfe_to_haa_z = [0; 0; obj.L1];
            
            xr = xr + hfe_to_haa_z;
            
            % joint2, joint3 를 찾는 부분
            
            tmp1 = xr(1)^2 + xr(3)^2;
            
            lu = obj.L2;
            ll = obj.L3;
            
            alpha = atan2(-xr(3), xr(1)) - 0.5*pi;
            
            some_random_value_for_beta = (lu^2+tmp1-ll^2)/(2.*lu*sqrt(tmp1));
            beta = acos(some_random_value_for_beta);
            
            % if (some_random_value_for_beta > 1) 
            %     some_random_value_for_beta = 1;
            % end
            % if (some_random_value_for_beta < -1)
            %     some_random_value_for_beta = -1;
            % end
            
            q_HFE_bf = alpha + beta;
            q_HFE_br = q_HFE_bf;
            
            some_random_value_for_gamma = (ll^2+lu^2-tmp1)/(2.*ll*lu);
            
            % if (some_random_value_for_gamma > 1) 
            %     some_random_value_for_gamma = 1;
            % end
            % if (some_random_value_for_gamma < -1)
            %     some_random_value_for_gamma = -1;
            % end
            
            gamma  = acos(some_random_value_for_gamma);
            
            q_KFE_bf = gamma - pi;
            q_KFE_br = q_KFE_bf;
            
            if legIdx == 0 || legIdx == 2 % forward
                joint1 = q_HAA_bf;
                joint2 = q_HFE_bf;
                joint3 = q_KFE_bf;
            elseif legIdx == 1 || legIdx == 3% backward
                joint1 = q_HAA_br;
                joint2 = -q_HFE_br;
                joint3 = -q_KFE_br;
            end
        end
    end
end

