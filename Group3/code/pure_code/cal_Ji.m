function Ji = cal_Ji(joint,k,j)
% k is the Jacobian column we wanna calculate
% j is the total number of joint
Kinem = eye(4,4);
if k == 1 
    b_i_1 = [0;0;1];
else
    for i = 1:k-1
        Kinem = Kinem*joint(i).rot;
    end
    b_i_1 = Kinem(1:3,3);
end
% b:3x1

if joint(k).type1 == 1 % if translation
    Ji = [b_i_1;0;0;0];% 6x1
end

if joint(k).type1 == 2 % if rotation
    r_i_1 = Kinem(1:3,4); % r:3x1
    Kinem1 = eye(4,4);
    P_end_effector = eye(4,4);
    for i = 1:j
        P_end_effector = P_end_effector * joint(i).rot;
    end
    P_end_effector = P_end_effector(1:3,4);

    r = P_end_effector - r_i_1;
    Ji = [b_i_1(2)*r(3)-b_i_1(3)*r(2);-b_i_1(1)*r(3)+b_i_1(3)*r(1);b_i_1(1)*r(2)-b_i_1(2)*r(1);b_i_1];
end