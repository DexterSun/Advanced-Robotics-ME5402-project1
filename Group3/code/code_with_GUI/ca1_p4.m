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
P_0_t = simplify(Kinem(1:3,4));
x0 = [0 0 0];
% double(subs(P_0_t,[theta1,theta2,theta3],x0))
options = optimoptions(@fsolve,'OptimalityTolerance',1e-8);
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

for i = 1:10
    bk1(i) = double(simplify(subs(q_dot(1),t,i-1)));
%     bk2(i) = double(subs(q_dot(2),t,i-1));
    bk3(i) = double(subs(q_dot(3),t,i-1));
end
close all
figure;
plot(1:10,bk1)
figure;
plot(1:10,bk3)
figure;
plot(1:10,bk3)
hold off

% position of point A
j = 1;
Pt1 = subs(Pt,[theta1,theta2,theta3],[theta_1,initial_thetas(2),initial_thetas(3)]);
for i = 1:pi/30:pi
    bpt1(j) = double(subs(Pt1(1),t,i-1));
    bpt2(j) = double(subs(Pt1(2),t,i-1));
    j = j+1;
end
plot(bpt1,bpt2)
%%
function rot = DH_rot(joint)
alpha = joint.DH(1);
a = joint.DH(2);
d = joint.DH(3);
theta = joint.DH(4);
rot = [cos(theta),  -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta);
       sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),  a*sin(theta);
       0,  sin(alpha),  cos(alpha), d ;
       0,  0,  0,  1 ];
rot2 = [1 0 0 a;
        0 cos(alpha) -sin(alpha) 0; 
        0 sin(alpha) cos(alpha) 0;
        0 0 0 1];
rot1 = [cos(theta) -sin(theta) 0 0;
        sin(theta) cos(theta) 0 0;
        0 0 1 d
        0 0 0 1];
end