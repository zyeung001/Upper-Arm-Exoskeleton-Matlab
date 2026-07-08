function R = rotX(a)
%ROTX Rotation matrix for a rotation of angle a (radians) about the x-axis.
c = cos(a); s = sin(a);
R = [1 0 0;
     0 c -s;
     0 s  c];
end
