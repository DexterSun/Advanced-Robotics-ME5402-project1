function rotation = rot(axis,angle)
if axis == 'x'
    rotation = [1,0,0;
                0,cos(angle),-sin(angle);
                0,sin(angle), cos(angle)];
else axis == 'z'
    rotation = [cos(angle),-sin(angle),0;
                sin(angle), cos(angle),0;
                0,0,1];
else axis == 'y'
    rotation = [cos(angle),0,sin(angle);
                0,1,0;
                -sin(angle),0,cos(angle)];
end