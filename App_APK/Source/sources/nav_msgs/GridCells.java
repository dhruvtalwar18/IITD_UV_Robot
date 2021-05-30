package nav_msgs;

import geometry_msgs.Point;
import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface GridCells extends Message {
    public static final String _DEFINITION = "#an array of cells in a 2D grid\nHeader header\nfloat32 cell_width\nfloat32 cell_height\ngeometry_msgs/Point[] cells\n";
    public static final String _TYPE = "nav_msgs/GridCells";

    float getCellHeight();

    float getCellWidth();

    List<Point> getCells();

    Header getHeader();

    void setCellHeight(float f);

    void setCellWidth(float f);

    void setCells(List<Point> list);

    void setHeader(Header header);
}
