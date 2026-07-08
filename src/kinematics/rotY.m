function R = rotY(a)
%ROTY Rotation matrix for a rotation of angle a (radians) about the y-axis.
c = cos(a); s = sin(a);
R = [ c 0 s;
      0 1 0;
     -s 0 c];
end
