function Figure=q4(iii)
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
% Jacobian derivation
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

% initializing joint variables
alpha = atan(100/150);
c = sqrt(150^2+100^2);
beta = pi/4-alpha;
a = c*sin(beta);% the rotating radius of tool end
h = c*cos(beta);
zz = 400+h; % this is the distance between frame 0 and the working surface
% then based on the parameters and representations, write a function to
% calculate the initial thetas
Kinem = eye(4,4);
for i = 1:4
    Kinem = Kinem*joint(i).rot;
end
R_0_t = Kinem(1:3,1:3);
P_0_t = simplify(Kinem(1:3,4))
x0 = [pi/6 pi/6 pi/6];
% double(subs(P_0_t,[theta1,theta2,theta3],x0))
options = optimoptions(@fsolve,'OptimalityTolerance',1e-8)
ans = fsolve(@buzhi,x0,options);
initial_thetas = double(ans)   % set the inital point at (0,100,250) and get the theta values
Ptsol = double(subs(P_0_t,[theta1,theta2,theta3],ans)) % inverse proof
double(Ptsol-[0; a; zz])
%%%%%%%%%%%%%%%%%%%

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
for i = 1:pi/30:2*pi+1
    bk1(j) = double(simplify(subs(q_dot(1),t,i-1)));
    bk2(j) = double(subs(q_dot(2),t,i-1));
    bk3(j) = double(subs(q_dot(3),t,i-1));
    j = j+1;
end

q_1(1)=initial_thetas(1);
q_2(1)=initial_thetas(2);
q_3(1)=initial_thetas(3);
for i =1:j-1
    q_1(i+1)=q_1(i)+bk1(i)* pi/30;
    q_2(i+1)=q_2(i)+bk2(i)* pi/30;
    q_3(i+1)=q_3(i)+bk3(i)* pi/30;
end


for i=1:j-1
    torque_1(i) = joint_force(1);
    torque_2(i)= joint_force(2);
    torque_3(i) = joint_force(3);
end

% configuration
T1 = joint(1).rot;
P1 = joint(1).rot(1:3,4);
R1 = joint(1).rot(1:3,1:3);
T2 = joint(1).rot*joint(2).rot;
P2 = T2(1:3,4);
R2 = T2(1:3,1:3);
T3 = T2*joint(3).rot;
P3 = T3(1:3,4);
R3 = T3(1:3,1:3);
T4 = T3*joint(4).rot;
P4 = T4(1:3,4);
R4 = T4(1:3,1:3);
T1 = subs(T1,theta1,theta_1);
T2 = subs(T2,[theta1,theta2],[theta_1,initial_thetas(2)]);
T3 = subs(T3,[theta1,theta2,theta3],[theta_1,initial_thetas(2),initial_thetas(3)]);
T4 = subs(T4,[theta1,theta2,theta3],[theta_1,initial_thetas(2),initial_thetas(3)]);
% % speed of each joint
figure('visible','off');
plot(0:length(bk1)-1,bk1,'DisplayName','joint1 rates','LineWidth',2)
hold on
plot(0:length(bk2)-1,bk2,'DisplayName','joint2 rates','LineStyle','-','LineWidth',2)
hold on 
plot(0:length(bk3)-1,bk3,'o','DisplayName','joint3 rates')
legend('Location','northwest')
title('joint rates versus time')
xlabel('time') 
ylabel('joint rates') 
axis([0,60,-1,2])
hold off
Figure1=getframe(gcf);
Figure1 = frame2im(Figure1);

% theta of each joint
figure('visible','off');
plot(0:length(q_1)-1,q_1,'DisplayName','joint1 theta','LineWidth',2)
hold on
plot(0:length(q_2)-1,q_2,'DisplayName','joint2 theta','LineWidth',2)
hold on
plot(0:length(q_3)-1,q_3,'DisplayName','joint3 theta','LineWidth',2)
legend('Location','northwest')
title('joint angles versus time')
xlabel('time') 
ylabel('joint angles') 
hold off
Figure2=getframe(gcf);
Figure2 = frame2im(Figure2);

% torque of each joint
figure('visible','off');
plot(0:length(torque_1)-1,torque_1,'DisplayName','joint1 torque','LineWidth',2)
hold on
plot(0:length(torque_2)-1,torque_2,'DisplayName','joint2 torque','LineWidth',2)
hold on
plot(0:length(torque_3)-1,torque_3,'DisplayName','joint3 torque','LineWidth',2)
legend('Location','northwest')
title('joint torques versus time')
xlabel('time') 
ylabel('joint torques') 
axis([0,60,-1,1])
hold off
Figure3=getframe(gcf);
Figure3 = frame2im(Figure3);
if iii == 1
Figure = Figure1;
end
if iii == 2
    Figure = Figure2;
end
if iii == 3
Figure = Figure3;
end
