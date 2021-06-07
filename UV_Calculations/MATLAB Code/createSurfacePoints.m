walls = [[1, 0]; [0, 1]; [-1, 0]; [0, -1]];
surfacePointList = [];
for w = 1:length(walls)
    wall = walls(w, :);
    x = cuboidSide*wall(1)/2;
    y = cuboidSide*wall(2)/2;
    if x == 0
        for x = -cuboidSide/2+cuboidResolution:cuboidResolution:cuboidSide/2
            for z = -cuboidHeight/2+cuboidResolution:cuboidResolution:cuboidHeight/2
                surfacePointList = [surfacePointList; [x y z]];
            end
        end
    else
        for y = -cuboidSide/2+cuboidResolution:cuboidResolution:cuboidSide/2
            for z = -cuboidHeight/2+cuboidResolution:cuboidResolution:cuboidHeight/2
                surfacePointList = [surfacePointList; [x y z]];
            end
        end
    end
end