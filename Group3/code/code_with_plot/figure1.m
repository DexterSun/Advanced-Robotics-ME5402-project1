plot3(0,0,0)
grid on
xlabel('axis X')
ylabel('axis Y')
zlabel('axis Z')
hold on
%%
plot3(0,0,0)
theta =pi/4
%axis([-5,5,-5,5,-5,5])
axis on
grid on 
hold on
quiver3(1,1,1,1,0,0,'red')
hold on
quiver3(1,1,1,0,1,0,'red')
hold on
quiver3(1,1,1,0,0,1,'red')
quiver3(0,0,0,cos(theta),-sin(theta),0,'green')
hold on
quiver3(0,0,0,sin(theta),cos(theta),0,'green')
hold on
quiver3(0,0,0,0,0,1,'green')
