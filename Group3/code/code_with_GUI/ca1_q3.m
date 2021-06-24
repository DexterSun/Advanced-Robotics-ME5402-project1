function joint_force3=ca1_q3(Theta1,D2,D3,F1,F2,F3,N1,N2,N3)
syms theta1 l1 d2 d3
% joint 1
% joint(1).DH = [theta1 l1 0 0];
joint(1).DH = [0 0 l1 theta1];
joint(1).type1 = 2; % 1:trans 2:rot
% joint 2
% joint(2).DH = [pi/2 d2 0 pi/2];
joint(2).DH = [pi/2 0 d2 pi/2];
joint(2).type1 = 1;
% joint 3
% joint(3).DH = [0 d3 0 0];
joint(3).DH = [0 0 d3 0];
joint(3).type1 = 1;
for i = 1:3
    joint(i).rot = DH_rot(joint(i));
end
J = zeros(6,1);
for i = 1:3
    J = [J,cal_Ji(joint,i,3)];
end
J = J(:,2:4)

syms f1 f2 f3 n1 n2 n3
F = [f1;f2;f3;n1;n2;n3];
joint_force = J.'*F

J3 = subs(J,[theta1,d2,d3],[Theta1*pi/180,D2,D3]);
F3 = subs(F,[f1,f2,f3,n1,n2,n3],[F1,F2,F3,N1,N2,N3]);
joint_force3 = double(J3.'*F3)


