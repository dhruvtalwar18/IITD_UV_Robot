function out = calcIrradiance(radius, phi, theta, alpha)
out = phi*cos(theta)*cos(theta)*cos(alpha)*cos(alpha)/(4*pi*radius*radius);
end
