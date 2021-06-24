function angles = inverse_kinematics(matrix)

l0 = 500;
l1 = 500;
l2 = 400;
l3 = 150;
l_offset = 200;
syms theta1 theta2 theta3 theta4 theta5 theta6 % joint variables
% syms l0 l1 l2 l3 loffset
% need analytical matrices first
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

T=matrix;

Kinem = eye(4,4);
for i = 1:4
    Kinem = Kinem*joint(i).rot;
end
T04 = Kinem; % wrist

Pw04 = T04(1:3,4);
Pw64 = T(1:3,4)-l3*T(1:3,3);

ans = solve(-sin(theta1)*Pw64(1)+cos(theta1)*Pw64(2)==l_offset, theta1);
theta1_1 = real(double(ans(1)));
theta1_2 = real(double(ans(2)));
if theta1_1>=0 && theta1_1<=2*pi
    thetav1 = theta1_1;
else
    thetav1 = theta1_2;
end
A=cos(thetav1)*Pw64(1)+sin(thetav1)*Pw64(2);
B=(A^2+l1^2+(l0-Pw64(3))^2-l2^2)/(2*l1);
theta2 = atan(A/(l0-Pw64(3)))-atan(B/(A^2+(l0-Pw64(3))^2-B^2)^0.5);
if theta2 <= 0
    thetav2 = theta2+pi;
else
    thetav2 = theta2;
end
theta3=atan((A-l1*cos(thetav2))/(l0+l1*sin(thetav2)-Pw64(3)))-thetav2;
if theta3 <= 0
    thetav3 = theta3+pi;
else
    thetav3 = theta3;
end
theta4=atan((T(1,3)*sin(thetav1)-T(2,3)*cos(thetav1))/(T(1,3)*cos(thetav1)*cos(thetav1+thetav2)+T(2,3)*sin(thetav1)*cos(thetav1+thetav2)+T(3,3)*sin(thetav1+thetav2)));
if theta4 <= 0
    thetav4 = theta4+pi;
else
    thetav4 = theta4;
end
theta5=acos(T(1,3)*cos(thetav1)*sin(thetav1+thetav2)+T(2,3)*sin(thetav1)*sin(thetav1+thetav2)-T(3,3)*cos(thetav1+thetav2));
if theta5 <= 0
    thetav5 = theta5+pi;
else
    thetav5 = theta5;
end
theta6=atan((T(1,2)*cos(thetav1)*sin(thetav1+thetav2)+T(2,2)*sin(thetav1)*sin(thetav1+thetav2)-T(3,2)*cos(thetav1+thetav2))/-(T(1,1)*cos(thetav1)*sin(thetav1+thetav2)+T(2,1)*sin(thetav1)*sin(thetav1+thetav2)-T(3,1)*cos(thetav1+thetav2)));
if theta6 <= 0
    thetav6 = theta6+pi;
else
    thetav6 = theta6;
end

if imag(roundn(thetav1,-2))~=0 || imag(roundn(thetav2,-2))~=0 || imag(roundn(thetav3,-2))~=0 || imag(roundn(thetav4,-2))~=0 || imag(roundn(thetav5,-2))~=0 || imag(roundn(thetav6,-2))~=0
   angles = [];
else
   angles = [round(thetav1*180/pi)
             round(thetav2*180/pi)
             round(thetav3*180/pi)
             round(thetav4*180/pi)
             round(thetav5*180/pi)
             round(thetav6*180/pi)];
       
end

       
