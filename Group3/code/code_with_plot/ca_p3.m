%% 3
syms theta1 l1 d2 d3
theta1 = 0;
l1 = 1;
d2 = 1;
d3 = 3;
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
    [joint(i).rot,joint(i).rot1,joint(i).rot2]= DH_rot(joint(i));
end


Kinem = eye(4,4);
Plotaxis(Kinem,0)
X=[Kinem(1,4)];
Y=[Kinem(2,4)];
Z=[Kinem(3,4)];
for i = 1:3
    if joint(i).DH(3) ~= 0
        PlotCuboid([Kinem(1,4),Kinem(2,4),Kinem(3,4)],[0.5,0.5,joint(i).DH(3)],[Kinem(1,1:3);Kinem(2,1:3);Kinem(3,1:3)],i * 10)   
    end
    Kinem = Kinem*joint(i).rot1;
    X=[X,Kinem(1,4)];
    Y=[Y,Kinem(2,4)];
    Z=[Z,Kinem(3,4)];
    Kinem = Kinem*joint(i).rot2;
    if joint(i).DH(2) ~= 0
        PlotCuboid([Kinem(1,4),Kinem(2,4),Kinem(3,4)],[joint(i).DH(2),0.5,0.5],[Kinem(1,1:3);Kinem(2,1:3);Kinem(3,1:3)],i * 10)
    end
    X=[X,Kinem(1,4)];
    Y=[Y,Kinem(2,4)];
    Z=[Z,Kinem(3,4)];
    
    plot3(Kinem(1,4),Kinem(2,4),Kinem(3,4),'.','MarkerSize', 30,'color',[1,0,0])
    Plotaxis(Kinem,i)
end
DH_draw(X,Y,Z)
%%
function [rot,rot1,rot2] = DH_rot(joint)
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
%%
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
end
%%
function [orgX, orgY, orgZ] = Plotaxis(Trans,i)
% This function takes in the Transformation matrix as a parameter and draws
% the coordinate system using it. It also returns the origin of the newly
% created coordinate system.

%% Handle errors (if any) due to incorrect dimension of Parameter
[r, c] = size(Trans);
if (r ~= 4 || c ~= 4)
    error('Invalid Argument. Transformation Matrix not of correct dimension.');
end

%% Plot the robot configuration using the current transformation matrices

orgX = Trans(1,4);
orgY = Trans(2,4);
orgZ = Trans(3,4);
X = quiver3(orgX,orgY,orgZ,Trans(1,1),Trans(2,1),Trans(3,1),'r');
hold on
Y = quiver3(orgX,orgY,orgZ,Trans(1,2),Trans(2,2),Trans(3,2),'g');
hold on
Z = quiver3(orgX,orgY,orgZ,Trans(1,3),Trans(2,3),Trans(3,3),'b');
hold on
str_x = sprintf('X_%d-Axis',i);
str_y = sprintf('Y_%d-Axis',i);
str_z = sprintf('Z_%d-Axis',i);
text_x = text(orgX + Trans(1,1), orgY + Trans(2, 1), orgZ + Trans(3, 1),str_x , 'color', 'red','FontSize',8);
text_y = text(orgX + Trans(1,2), orgY + Trans(2, 2), orgZ + Trans(3, 2), str_y, 'color', 'green','FontSize',8);
text_z = text(orgX + Trans(1,3), orgY + Trans(2, 3), orgZ + Trans(3, 3), str_z, 'color', 'blue','FontSize',8);

end
%%
function [] = DH_draw(X, Y, Z)

[~, c(1)] = size(X);
[~, c(2)] = size(Y);
[~, c(3)] = size(Z);

if (c(1) ~= c(2) || c(1) ~= c(3))
    error('Invalid arguments. Size of all the arguments should be same')
    return
end

if (c(1) < 2)
    error('The input coordinate arrays should have atleast 2 entries')
end

for i = 1:(c(1)-1)
    plot=plot3([X(i), X(i+1)], [Y(i), Y(i+1)], [Z(i), Z(i+1)],'color',[0.8,0.8,0.8],'LineWidth',8);
    axis([-4,4,-4,4,0,4])
    plot.Color(4)=0.5
    hold on
end
end
%%
function PlotCuboid(originPoint,cuboidSize,direction,j)
% origin: originPoint, 1*3 row vector
% cuboidSize: size in relative [x,y,z] direction(represented in local frame)
% direction: x,y,z unit vector (R matrix of frame representation)
% i: to distinguish different links with colors

% calculate 8 corner points
vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
vet = vertexIndex.*cuboidSize % 8*3
for i = 1:8
    plusvec1 = direction*(vet(i,:).');
    plusvec(i,1:3) = plusvec1.'
end
vertex=originPoint+plusvec;

% define the characteristic point of each face
facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];

% define the color of 8 corner points

color = [1/j;1/j;1/j;1/j;1/j;1/j;1/j;1/j];

% plot
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha', 0.05);
view(3);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
fig=gcf;
fig.Color=[1 1 1];
fig.NumberTitle='off';
end