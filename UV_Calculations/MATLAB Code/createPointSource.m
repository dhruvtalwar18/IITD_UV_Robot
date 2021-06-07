% For each Lamp
pointSourceList = [];
for deg = 0:2*pi/noOfLamps:2*pi-.01
    x = lampRadius*sin(deg);
    y = lampRadius*cos(deg);
    % For each element
    for height = -lengthLamp/2 + LampResolution/2:LampResolution:lengthLamp/2
        z = height;
        pointSourceList = [pointSourceList; [x y z]];
    end
end
