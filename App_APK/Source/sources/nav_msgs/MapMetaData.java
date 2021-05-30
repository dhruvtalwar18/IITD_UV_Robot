package nav_msgs;

import geometry_msgs.Pose;
import org.ros.internal.message.Message;
import org.ros.message.Time;

public interface MapMetaData extends Message {
    public static final String _DEFINITION = "# This hold basic information about the characterists of the OccupancyGrid\n\n# The time at which the map was loaded\ntime map_load_time\n# The map resolution [m/cell]\nfloat32 resolution\n# Map width [cells]\nuint32 width\n# Map height [cells]\nuint32 height\n# The origin of the map [m, m, rad].  This is the real-world pose of the\n# cell (0,0) in the map.\ngeometry_msgs/Pose origin";
    public static final String _TYPE = "nav_msgs/MapMetaData";

    int getHeight();

    Time getMapLoadTime();

    Pose getOrigin();

    float getResolution();

    int getWidth();

    void setHeight(int i);

    void setMapLoadTime(Time time);

    void setOrigin(Pose pose);

    void setResolution(float f);

    void setWidth(int i);
}
