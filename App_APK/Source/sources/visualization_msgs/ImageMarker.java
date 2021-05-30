package visualization_msgs;

import geometry_msgs.Point;
import java.util.List;
import org.ros.internal.message.Message;
import org.ros.message.Duration;
import std_msgs.ColorRGBA;
import std_msgs.Header;

public interface ImageMarker extends Message {
    public static final byte ADD = 0;
    public static final byte CIRCLE = 0;
    public static final byte LINE_LIST = 2;
    public static final byte LINE_STRIP = 1;
    public static final byte POINTS = 4;
    public static final byte POLYGON = 3;
    public static final byte REMOVE = 1;
    public static final String _DEFINITION = "uint8 CIRCLE=0\nuint8 LINE_STRIP=1\nuint8 LINE_LIST=2\nuint8 POLYGON=3\nuint8 POINTS=4\n\nuint8 ADD=0\nuint8 REMOVE=1\n\nHeader header\nstring ns\t\t# namespace, used with id to form a unique id\nint32 id          \t# unique id within the namespace\nint32 type        \t# CIRCLE/LINE_STRIP/etc.\nint32 action      \t# ADD/REMOVE\ngeometry_msgs/Point position # 2D, in pixel-coords\nfloat32 scale\t \t# the diameter for a circle, etc.\nstd_msgs/ColorRGBA outline_color\nuint8 filled\t\t# whether to fill in the shape with color\nstd_msgs/ColorRGBA fill_color # color [0.0-1.0]\nduration lifetime       # How long the object should last before being automatically deleted.  0 means forever\n\n\ngeometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POINTS/etc., 2D in pixel coords\nstd_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.";
    public static final String _TYPE = "visualization_msgs/ImageMarker";

    int getAction();

    ColorRGBA getFillColor();

    byte getFilled();

    Header getHeader();

    int getId();

    Duration getLifetime();

    String getNs();

    ColorRGBA getOutlineColor();

    List<ColorRGBA> getOutlineColors();

    List<Point> getPoints();

    Point getPosition();

    float getScale();

    int getType();

    void setAction(int i);

    void setFillColor(ColorRGBA colorRGBA);

    void setFilled(byte b);

    void setHeader(Header header);

    void setId(int i);

    void setLifetime(Duration duration);

    void setNs(String str);

    void setOutlineColor(ColorRGBA colorRGBA);

    void setOutlineColors(List<ColorRGBA> list);

    void setPoints(List<Point> list);

    void setPosition(Point point);

    void setScale(float f);

    void setType(int i);
}
