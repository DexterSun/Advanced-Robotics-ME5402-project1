function Figure=plot_q2(theta1,theta2,theta3,theta4,theta5,theta6)

syms l0 l1 l2 l3 l_offset
l0 = 500;
l1 = 500;
l2 = 400;
l3 = 150;
l_offset = 200;
theta1 = theta1*pi/180;
theta2 = theta2*pi/180;
theta3 = theta3*pi/180;
theta4 = theta4*pi/180;
theta5 = theta5*pi/180;
theta6 = theta6*pi/180;
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
joint(4).type = 1; % trans:1 rot:2
% joint 5
joint(5).DH = [pi/2, 0, 0, theta5];
joint(5).type = 2; % trans:1 rot:2
% joint 6
joint(6).DH = [0, 0, l3, theta6];
joint(6).type = 2; % trans:1 rot:2
for i = 1:6
[joint(i).rot,joint(i).rot1,joint(i).rot2] = DH_rot(joint(i));
end
% till now we have defined the DH table and all transformation matrices
% we can get the kinematical function easily by multiplying them together
Kinem = eye(4,4);
Plotaxis(Kinem,0)
X=[Kinem(1,4)];
Y=[Kinem(2,4)];
Z=[Kinem(3,4)];
j=1;
for i = 1:6
    if joint(i).DH(3) ~= 0
        PlotCuboid([Kinem(1,4),Kinem(2,4),Kinem(3,4)],[50,50,joint(i).DH(3)],[Kinem(1,1:3);Kinem(2,1:3);Kinem(3,1:3)],j)   
        j = j+1;
    end
    Kinem = Kinem*joint(i).rot1;
    X=[X,Kinem(1,4)];
    Y=[Y,Kinem(2,4)];
    Z=[Z,Kinem(3,4)];
    if joint(i).DH(2) ~= 0
        PlotCuboid([Kinem(1,4),Kinem(2,4),Kinem(3,4)],[joint(i).DH(2),50,50],[Kinem(1,1:3);Kinem(2,1:3);Kinem(3,1:3)],j)
        j = j+1;
    end
    figure(1);
    plota=plot3(Kinem(1,4),Kinem(2,4),Kinem(3,4),'.','MarkerSize', 20,'color',[1,0,0]);
    Kinem = Kinem*joint(i).rot2;
    X=[X,Kinem(1,4)];
    Y=[Y,Kinem(2,4)];
    Z=[Z,Kinem(3,4)];
    plota=plot3(Kinem(1,4),Kinem(2,4),Kinem(3,4),'.','MarkerSize', 20,'color',[1,0,0]);
    % str_joint = sprintf('joint-%d',i);
    % text(Kinem(1,4),Kinem(2,4),Kinem(3,4)+100,str_joint);
    Plotaxis(Kinem,i)
end

PlotCuboid([Kinem(1,4),Kinem(2,4),Kinem(3,4)],[50,50,joint(i).DH(3)],[Kinem(1,1:3);Kinem(2,1:3);Kinem(3,1:3)],i * 10) 
Figure=getframe(gcf);
Figure = frame2im(Figure);
end
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
function [orgX, orgY, orgZ] = Plotaxis(Trans,i)
% This function takes in the Transformation matrix as a parameter and draws
% the coordinate system using it. It also returns the origin of the newly
% created coordinate system.

% Handle errors (if any) due to incorrect dimension of Parameter
[r, c] = size(Trans);
if (r ~= 4 || c ~= 4)
    error('Invalid Argument. Transformation Matrix not of correct dimension.');
end

%Plot the robot configuration using the current transformation matrices

orgX = Trans(1,4);
orgY = Trans(2,4);
orgZ = Trans(3,4);

X = quiver3(orgX,orgY,orgZ,100*Trans(1,1),100*Trans(2,1),100*Trans(3,1),'r');
hold on
Y = quiver3(orgX,orgY,orgZ,100*Trans(1,2),100*Trans(2,2),100*Trans(3,2),'g');
hold on
Z = quiver3(orgX,orgY,orgZ,100*Trans(1,3),100*Trans(2,3),100*Trans(3,3),'b');
hold on

str_x = sprintf('X_%d',i);
str_y = sprintf('Y_%d',i);
str_z = sprintf('Z_%d',i);
text_x = text(orgX + 100*Trans(1,1), orgY + 100*Trans(2, 1), orgZ + 100*Trans(3, 1), str_x, 'color', 'red','FontSize',8);
text_y = text(orgX + 100*Trans(1,2), orgY + 100*Trans(2, 2), orgZ + 100*Trans(3, 2), str_y, 'color', 'green','FontSize',8);
text_z = text(orgX + 100*Trans(1,3), orgY + 100*Trans(2, 3), orgZ + 100*Trans(3, 3), str_z, 'color', 'blue','FontSize',8);

end
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
    axis([-1000,1000,-1000,1000,0,1000])
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
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha', 0.25);
view(3);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
fig=gcf;
fig.Color=[1 1 1];
fig.NumberTitle='off';
end