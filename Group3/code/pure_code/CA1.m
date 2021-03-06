%% ME5404 Advanced Robotics
% CA part 1
clear all;
clc;


%% Problem 1 sub-problem 1a
% about y by alpha
% about x by beta
% define a function to give the rotation matrix
beta   = pi/4;     % about x axis
alpha  = pi/6;     % about y axis
gamma  = 0;  % about z axis
order = [1,2,3];  % order of rotation(sequence:beta,alpha,gamma) x-y-z
rot_matrix = y_x_rot(beta, alpha, gamma, order)  % in order x-y-zx

%% Problem 1 sub-problem 1b
% rotation matrix to XY
r11 = 0.866; r12 = -0.5; r13 = 0;
r21 = 0.5; r22 = 0.866; r23 = 0;
r31 = 0; r32 = 0; r33 = 1;
R = [r11,r12,r13;r21,r22,r23;r31,r32,r33];% easy to check and change
XYZ = rot_to_XYZ(R);
XYZ1 = XYZ(1,:) % solution 1 - sequence x-y-z
XYZ2 = XYZ(2,:) % solution 2 - sequence x-y-z
%% Problem 1 sub-problem 1b
% XYZ to rotation matrix
gamma  = pi/4   ; % x
beta   = pi/6   ; % y
alpha  = 0   ; % z
rot = xyz_to_rot(alpha,beta,gamma)

%% Problem 2 sub-problem 2 PUMA 600
% DH table of PUMA600
syms theta1 theta2 theta3 theta4 theta5 theta6 % joint variables
syms l0 l1 l2 l3 l_offset
% theta1 = pi/6;
% theta2 = pi/6;
% theta3 = pi/6;
% theta4 = pi/6;
% theta5 = pi/6;
% theta6 = pi/6;

% l0 = 500;
% l1 = 500;
% l2 = 400;
% l3 = 150;
% l_offset = 200;
% joint 1
joint(1).DH = [pi/2, 0, l0, theta1];
joint(1).type = 2; % trans:1 rot:2
% joint 2
joint(2).DH = [0, l1, -l_offset, theta2];
joint(2).type = 2; % trans:1 rot:2
% joint 3
joint(3).DH = [pi/2, 0, 0, theta3];
joint(3).type = 2; % trans:1 rot:2
% joint 4
joint(4).DH = [-pi/2, 0, l2, theta4];
joint(4).type = 2; % trans:1 rot:2
% joint 5
joint(5).DH = [pi/2, 0, 0, theta5];
joint(5).type = 2; % trans:1 rot:2
% joint 6
joint(6).DH = [0, 0, l3, theta6];
joint(6).type = 2; % trans:1 rot:2
for i = 1:6
    joint(i).rot = DH_rot(joint(i));
end
% till now we have defined the DH table and all transformation matrices
% we can get the kinematical function easily by multiplying them together
Kinem = eye(4,4);
for i = 1:6
    Kinem = Kinem*joint(i).rot;
end
Kinem
n = simplify(Kinem(1:3,1));
a = simplify(Kinem(1:3,2));
o = simplify(Kinem(1:3,3));
p = simplify(Kinem(1:3,4));

%% inverse kinematics method 1
T = [0.2522 , 0.1044 , 0.9620 , 719.3029;
     -0.7874 ,  -0.5558 ,   0.2667 , 602.9285;
     0.5625  , -0.8248 ,  -0.0580 , 541.2981;
       0         0         0    1.0000    ];
% fist move the inverse of T1 to the LHS of equation
lhs1 = simplify(inv(joint(1).rot))*T;
Kinem = eye(4,4);
for i = 2:6
    Kinem = Kinem*joint(i).rot;
end
Kinem = simplify(Kinem)
eq = subs(Kinem(3,4),sin(theta4),lhs1(3,3)/sin(theta5));
ans = solve(lhs1(3,4)==eq)
jointv1 = real(double(ans(2)));

% then move the inverse of T2,T3 to the LHS of the equation
lhs2 = simplify(subs(inv(joint(1).rot*joint(2).rot*joint(3).rot)*T,theta1,jointv1))
Kinem = eye(4,4);
for i = 4:6
    Kinem = Kinem*joint(i).rot;
end
fun = @root;
x0 = [pi/6,pi/6,pi/6,pi/6];
x = fsolve(fun,x0);
jointv2 = x(1);
jointv3 = x(2);
jointv4 = x(3);
jointv5 = x(4);
Kinem = subs(Kinem,[theta2,theta3,theta4,theta5],[jointv2,jointv3,jointv4,jointv5]);
lhs = subs(lhs2,[theta2,theta3],[jointv2,jointv3]);
ans = solve(lhs(1,1)==Kinem(1,1));
jointv6 = double(ans(1));
fprintf('\n the joint variables are:\n %s \n %s \n %s \n %s \n %s \n %s \n',...
    num2str(jointv1),num2str(jointv2)...
    ,num2str(jointv3) ,num2str(jointv4) ,num2str(jointv5) ,num2str(jointv6))



%% 3
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

J3 = subs(J,[theta1,d2,d3],[0,1,1]);
F3 = subs(F,[f1,f2,f3,n1,n2,n3],[1,2,3,0,0,0]);
joint_force3 = J3.'*F3

%% 4
% 1 & 2
syms theta1 theta2 theta3 real
syms f1 f2 f3 n1 n2 n3
% joint 1
joint(1).DH = [-pi/2 0 400 theta1];
joint(1).type1 = 2; % 1:trans 2:rot
% joint 2
joint(2).DH = [pi/2 0 0 theta2];
joint(2).type1 = 2;
% joint 3
joint(3).DH = [0 0 100 theta3];
joint(3).type1 = 2;
% tool working point A
joint(4).DH = [0 100 50 0];
% calculate rotation representations
for i = 1:4
    joint(i).rot = DH_rot(joint(i));
end
J = zeros(6,1);
for i = 1:3
    J = [J,cal_Ji(joint,i,3)];
end
J = simplify(J(:,2:4))
Kinem = eye(4,4);
for i = 1:3
    Kinem = Kinem*joint(i).rot;
end
R_0_3 = Kinem(1:3,1:3);
P_0_3 = Kinem(1:3,4);
Kinem = eye(4,4);
for i = 1:4
    Kinem = Kinem*joint(i).rot;
end
% initializing joint variables
alpha = atan(100/150);
c = sqrt(150^2+100^2);
beta = pi/4-alpha;
a = c*sin(beta);% the rotating radius of tool end
h = c*cos(beta);
zz = 400+h; % this is the distance between frame 0 and the working surface
% then based on the parameters and representations, write a function to
% calculate the initial thetas

R_0_t = Kinem(1:3,1:3);
P_0_t = simplify(Kinem(1:3,4))
x0 = [0 0 0];
% double(subs(P_0_t,[theta1,theta2,theta3],x0))
options = optimoptions(@fsolve,'OptimalityTolerance',1e-8)
ans = fsolve(@buzhi,x0,options);
initial_thetas = double(ans)   % set the inital point at (0,100,250) and get the theta values
Ptsol = double(subs(P_0_t,[theta1,theta2,theta3],ans)) % inverse proof
double(Ptsol-[0; a; zz])


% by observation, we notice that ft is parallel to z1
% force:
z1 = joint(1).rot(1:3,3);
ft = -z1;
fn = [0;0;-1];
ft_mag = 10;
fn_mag = 10;
f = ft_mag*ft + fn_mag*fn;
% torque:
nx3 = 0.04;
x3 = R_0_3(1:3,1);
n = nx3*x3;

Ft = double(subs([f;n],[theta1,theta2,theta3],initial_thetas)); % on tool
% now apply force transformation
lower_vec = double(subs(R_0_3*joint(4).rot(1:3,4),[theta1,theta2,theta3],initial_thetas));
upper_mat = cross_dot(lower_vec);
F3 = [eye(3,3),zeros(3,3);
    upper_mat,eye(3,3)]*Ft;
joint_force = vpa(double(subs(J.'*F3,[theta1,theta2,theta3],initial_thetas)/1000),5)
% here divide by 1000 because the unit of length is mm

% 3
% define velocity and position of tool
syms t real
V = [-a*cos(t);-a*sin(t);0];
X = simplify([-a*sin(t);a*cos(t);zz])
Pt = simplify(P_0_t);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tried to find analytical solution but failed
% eqns = [X(1)==Pt(1) X(2)==Pt(2) X(3)==Pt(3)]
% thet = solve(eqns,[theta1 theta2 theta3])
% failed but simplified by hand and get a relation
% sin(theta1-t) ~ f(theta2,theta3)
% so we fix theta2 and theta3 then theta1-t fixed
% this means that theta1 increases constantly with t
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% have defined that all parts rotates with z0
% then the angular velocity of tool is also along the negetive direction of
% z0 [0;0;-1] with angular velocity theta1_dot
% theta1_dot can be calculated by the velocity of tool end, omega = v/r = 1
sin_theta1_t = (6*sin(initial_thetas(2))+4*cos(initial_thetas(2))*cos(initial_thetas(3)))/sqrt(2);
theta_1 = asin(sin_theta1_t)+t;

PO3 = subs(P_0_3,[theta2,theta3],[initial_thetas(2),initial_thetas(3)]);
xy3 = PO3;
xy3m = subs(xy3,theta1,theta_1);
v3 = diff(xy3m,t);

J33 = simplify(subs(J,[theta1,theta2,theta3],[theta_1,initial_thetas(2),initial_thetas(3)]));
% if using J.'*inv(J*J.'), the result is with entries of inf. this is the
% singular point (guess), therefore using pinv
q_dot = simplify(subs(simplify(pinv(J33)*[v3;0;0;1]),theta1,theta_1))% here use pinv to calculate the pseudo inverse of J33
j = 1;
for i = 1:pi/30:pi+1
    bk1(j) = double(simplify(subs(q_dot(1),t,i-1)));
    bk2(j) = double(subs(q_dot(2),t,i-1));
    bk3(j) = double(subs(q_dot(3),t,i-1));
    j = j+1;
end
close all
figure;
plot(1:length(bk1),bk1)
figure;
plot(1:length(bk2),bk2)
figure;
plot(1:length(bk3),bk3)
hold off

% position of point A
j = 1;
Pt1 = subs(Pt,[theta1,theta2,theta3],[theta_1,initial_thetas(2),initial_thetas(3)]);
for i = 1:pi/30:pi+1
    bpt1(j) = double(subs(Pt1(1),t,i-1));
    bpt2(j) = double(subs(Pt1(2),t,i-1));
    j = j+1;
end
figure;
plot(bpt1,bpt2)