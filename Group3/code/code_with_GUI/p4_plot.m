function Figure=p4_plot(i)




 Figure = q4_plot(i);

end
 
function Figure=q4_plot(t)
global pic_num
global X
global Y
global Z
if t == 0
 pic_num = 1;
 X=[];
 Y=[];
 Z=[];
end

    fig =figure('visible','off');

    syms theta1 theta2 theta3 real
syms f1 f2 f3 n1 n2 n3
theta1 = -0.5008 + t;
theta2 = -0.6606;
theta3 = -0.3153;
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
    [joint(i).rot,joint(i).rot1,joint(i).rot2] = DH_rot(joint(i));
end
Kinem = eye(4,4);
Plotaxis(Kinem,0)
for i = 1:4
    if joint(i).DH(3) ~= 0
        PlotCuboid([Kinem(1,4),Kinem(2,4),Kinem(3,4)],[10,10,joint(i).DH(3)],[Kinem(1,1:3);Kinem(2,1:3);Kinem(3,1:3)],i)   
    end
    Kinem = Kinem*joint(i).rot1;
    if joint(i).DH(2) ~= 0
        PlotCuboid([Kinem(1,4),Kinem(2,4),Kinem(3,4)],[joint(i).DH(2),10,10],[Kinem(1,1:3);Kinem(2,1:3);Kinem(3,1:3)],i)
    end
    Kinem = Kinem*joint(i).rot2;
    plota=plot3(Kinem(1,4),Kinem(2,4),Kinem(3,4),'.','MarkerSize', 20,'color',[1,0,0]);

    Plotaxis(Kinem,i)
    axis equal
    axis([-100,100,-100,100,0,600])
end
    X=[X,Kinem(1,4)];
    Y=[Y,Kinem(2,4)];
    Z=[Z,Kinem(3,4)];
    plotb=scatter3(X,Y,Z);
    [X_1,Y_1]=meshgrid(-100:200:100,-100:200:100);
    Z_1 = X_1*0 + Kinem(3,4);
    surf(X_1,Y_1,Z_1,'facecolor','yellow','FaceAlpha', 0.05)

    drawnow;
    F = getframe;
    I = frame2im(F);

    [I,map] = rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,'p4.gif','gif','Loopcount',inf,'DelayTime',0.1);
    else 
        imwrite(I,map,'p4.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    pic_num = pic_num +1 ;
    set(gca,'ZDir','reverse');
    Figure=getframe(gcf);
    Figure = frame2im(Figure);
    clf(fig)
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