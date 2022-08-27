syms x y r l

th = acos((x^2+y^2+r^2+l^2)/(2*r*sqrt(x^2+y^2))) + atan(y/x);

phi = atan((y-r*sin(th)/(x-r*cos(th)))) - th;

x_f = r*cos(th) + l*cos(th + phi);
y_f = r*sin(th) + l*sin(th + phi);