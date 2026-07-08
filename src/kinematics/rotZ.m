function R = rotZ(a)
%ROTZ Rotation matrix for a rotation of angle a (radians) about the z-axis.
c = cos(a); s = sin(a);
R = [c -s 0;
     s  c 0;
     0  0 1];
end
