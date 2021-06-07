%% Units
% Length = Meter
% Power = Watt
% Time = Second
% Angle = Radians
% X Right | Y Out | Z Up
clc
clear
close all
%% Lamps
noOfLamps = 5;
lampRadius = .25;
lengthLamp = 1.1;
LampResolution = .01;
phiLamp = 11.2;
phi = phiLamp/(lengthLamp/LampResolution);
createPointSource;
%% Plane Surface
% Cuboid with center at 0, 0, 0
cuboidSide = 4;
cuboidHeight = 2;
cuboidResolution = .05;
createSurfacePoints;
%% Calculate Irradiance for each surface point
irradianceValues = zeros(length(surfacePointList), 1);
lenPointSourceList = length(surfacePointList);
for i = 1:length(pointSourceList)
    pointSource = pointSourceList(i, :);
    for j = 1:length(surfacePointList)
        surfacePoint = surfacePointList(j, :);
        [theta, alpha] = calcAngles(pointSource, surfacePoint);
        irradianceValues(j) = irradianceValues(j) + calcIrradiance(cuboidSide/2, phi, theta, alpha);
    end
end
surfacePointList = [surfacePointList ones(length(surfacePointList), 1) irradianceValues];
%% Visualise Lamps and Surface
scatter3(pointSourceList(:, 1), pointSourceList(:, 2), pointSourceList(:, 3))
hold on
scatter3(surfacePointList(:, 1), surfacePointList(:, 2), surfacePointList(:, 3), surfacePointList(:, 4), surfacePointList(:, 5))
colorbar
% close all