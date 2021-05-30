package visualization_msgs;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Vector3;
import java.util.List;
import org.ros.internal.message.Message;
import org.ros.message.Duration;
import std_msgs.ColorRGBA;
import std_msgs.Header;

public interface Marker extends Message {
    public static final byte ADD = 0;
    public static final byte ARROW = 0;
    public static final byte CUBE = 1;
    public static final byte CUBE_LIST = 6;
    public static final byte CYLINDER = 3;
    public static final byte DELETE = 2;
    public static final byte DELETEALL = 3;
    public static final byte LINE_LIST = 5;
    public static final byte LINE_STRIP = 4;
    public static final byte MESH_RESOURCE = 10;
    public static final byte MODIFY = 0;
    public static final byte POINTS = 8;
    public static final byte SPHERE = 2;
    public static final byte SPHERE_LIST = 7;
    public static final byte TEXT_VIEW_FACING = 9;
    public static final byte TRIANGLE_LIST = 11;
    public static final String _DEFINITION = "# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz\n\nuint8 ARROW=0\nuint8 CUBE=1\nuint8 SPHERE=2\nuint8 CYLINDER=3\nuint8 LINE_STRIP=4\nuint8 LINE_LIST=5\nuint8 CUBE_LIST=6\nuint8 SPHERE_LIST=7\nuint8 POINTS=8\nuint8 TEXT_VIEW_FACING=9\nuint8 MESH_RESOURCE=10\nuint8 TRIANGLE_LIST=11\n\nuint8 ADD=0\nuint8 MODIFY=0\nuint8 DELETE=2\nuint8 DELETEALL=3\n\nHeader header                        # header for time/frame information\nstring ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object\nint32 id \t\t                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later\nint32 type \t\t                       # Type of object\nint32 action \t                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects\ngeometry_msgs/Pose pose                 # Pose of the object\ngeometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)\nstd_msgs/ColorRGBA color             # Color [0.0-1.0]\nduration lifetime                    # How long the object should last before being automatically deleted.  0 means forever\nbool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep\n\n#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)\ngeometry_msgs/Point[] points\n#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)\n#number of colors must either be 0 or equal to the number of points\n#NOTE: alpha is not yet used\nstd_msgs/ColorRGBA[] colors\n\n# NOTE: only used for text markers\nstring text\n\n# NOTE: only used for MESH_RESOURCE markers\nstring mesh_resource\nbool mesh_use_embedded_materials\n";
    public static final String _TYPE = "visualization_msgs/Marker";

    int getAction();

    ColorRGBA getColor();

    List<ColorRGBA> getColors();

    boolean getFrameLocked();

    Header getHeader();

    int getId();

    Duration getLifetime();

    String getMeshResource();

    boolean getMeshUseEmbeddedMaterials();

    String getNs();

    List<Point> getPoints();

    Pose getPose();

    Vector3 getScale();

    String getText();

    int getType();

    void setAction(int i);

    void setColor(ColorRGBA colorRGBA);

    void setColors(List<ColorRGBA> list);

    void setFrameLocked(boolean z);

    void setHeader(Header header);

    void setId(int i);

    void setLifetime(Duration duration);

    void setMeshResource(String str);

    void setMeshUseEmbeddedMaterials(boolean z);

    void setNs(String str);

    void setPoints(List<Point> list);

    void setPose(Pose pose);

    void setScale(Vector3 vector3);

    void setText(String str);

    void setType(int i);
}
