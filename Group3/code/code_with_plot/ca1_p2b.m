%% inverse kinematics
T = [0.2500 , 0.4330 , 0.8660 , 779.9038;
     -0.4330 ,  -0.7500  ,  0.5000 , 681.2178;
     0.8660 ,  -0.5000 ,  -0.0000 , 500.0000;
       0    ,     0    ,    0   ,    1    ];
% fist move the inverse of T1 to the LHS of equation
lhs1 = simplify(inv(joint(1).rot))*T;
Kinem = eye(4,4);
for i = 2:6
    Kinem = Kinem*joint(i).rot;
end
ans = solve(lhs1(3,3)==Kinem(3,3));
jointv1 = double(ans(2));
lhs1 = subs(lhs1,'theta1',jointv1);
ans = solve(lhs1(3,1)==Kinem(3,1));
jointv6 = double(ans(1));

% then move the inverse of T2 to the LHS of the equation
lhs2 = subs(inv(joint(1).rot*joint(2).rot)*T,[theta1,theta6],[jointv1,jointv6]);
Kinem = eye(4,4);
for i = 3:6
    Kinem = Kinem*joint(i).rot;
end
ans = solve(lhs2(3,1)==Kinem(3,1));
jointv2 = double(ans(1));

% then move the inverse of T3 to the LHS of the equation
lhs3 = simplify(subs(inv(joint(1).rot*joint(2).rot*joint(3).rot)*T,[theta1,theta2,theta6],[jointv1,jointv2,jointv6]));
% lhs3 = subs(lhs3,[theta1,theta2],[jointv1,jointv2])
lhs3 = simplify(lhs3);
Kinem = eye(4,4);
for i = 4:6
    Kinem = Kinem*joint(i).rot;
end
rr = Kinem(2,1);
ll = double(lhs3(2,1));
ans = solve(ll==rr);
jointv3 = double(ans(1));
rr = Kinem(1,3);
ll = double(subs(lhs3(1,3),theta3,jointv3));
ans = solve(ll==rr);
jointv5 = double(ans(1));
rr = subs(Kinem(3,4),theta5,jointv5);
ll = double(subs(lhs3(3,4),theta3,jointv3));
ans = solve(ll==rr);
jointv4 = double(ans);
str = ['the joint variables are:' num2str(jointv1) num2str(jointv2)...
    num2str(jointv3) num2str(jointv4) num2str(jointv5) num2str(jointv6)];
fprintf('\n the joint variables are:\n %s \n %s \n %s \n %s \n %s \n %s \n',...
    num2str(jointv1),num2str(jointv2)...
    ,num2str(jointv3) ,num2str(jointv4) ,num2str(jointv5) ,num2str(jointv6))