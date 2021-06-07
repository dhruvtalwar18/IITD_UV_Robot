function [theta, alpha] = calcAngles(pointSource, surfacePoint)
% theta = atan((pointSource(3) - surfacePoint(3))/(pointSource(1) - surfacePoint(1)));
% alpha = atan((pointSource(2) - surfacePoint(2))/(pointSource(1) - surfacePoint(1)));
v1 = pointSource;
v2 = surfacePoint - pointSource;
t1 = [v1(1), v1(3)];
t2 = [v2(1), v2(3)];
theta = acos(dot(t1, t2)/(norm(t1)+norm(t2)));
a1 = [v1(1), v1(2)];
a2 = [v2(1), v2(2)];
alpha = acos(dot(a1, a2)/(norm(a1)+norm(a2)));
end