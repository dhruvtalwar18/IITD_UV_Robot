package visualization_msgs;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface InteractiveMarkerFeedback extends Message {
    public static final byte BUTTON_CLICK = 3;
    public static final byte KEEP_ALIVE = 0;
    public static final byte MENU_SELECT = 2;
    public static final byte MOUSE_DOWN = 4;
    public static final byte MOUSE_UP = 5;
    public static final byte POSE_UPDATE = 1;
    public static final String _DEFINITION = "# Time/frame info.\nHeader header\n\n# Identifying string. Must be unique in the topic namespace.\nstring client_id\n\n# Feedback message sent back from the GUI, e.g.\n# when the status of an interactive marker was modified by the user.\n\n# Specifies which interactive marker and control this message refers to\nstring marker_name\nstring control_name\n\n# Type of the event\n# KEEP_ALIVE: sent while dragging to keep up control of the marker\n# MENU_SELECT: a menu entry has been selected\n# BUTTON_CLICK: a button control has been clicked\n# POSE_UPDATE: the pose has been changed using one of the controls\nuint8 KEEP_ALIVE = 0\nuint8 POSE_UPDATE = 1\nuint8 MENU_SELECT = 2\nuint8 BUTTON_CLICK = 3\n\nuint8 MOUSE_DOWN = 4\nuint8 MOUSE_UP = 5\n\nuint8 event_type\n\n# Current pose of the marker\n# Note: Has to be valid for all feedback types.\ngeometry_msgs/Pose pose\n\n# Contains the ID of the selected menu entry\n# Only valid for MENU_SELECT events.\nuint32 menu_entry_id\n\n# If event_type is BUTTON_CLICK, MOUSE_DOWN, or MOUSE_UP, mouse_point\n# may contain the 3 dimensional position of the event on the\n# control.  If it does, mouse_point_valid will be true.  mouse_point\n# will be relative to the frame listed in the header.\ngeometry_msgs/Point mouse_point\nbool mouse_point_valid\n";
    public static final String _TYPE = "visualization_msgs/InteractiveMarkerFeedback";

    String getClientId();

    String getControlName();

    byte getEventType();

    Header getHeader();

    String getMarkerName();

    int getMenuEntryId();

    Point getMousePoint();

    boolean getMousePointValid();

    Pose getPose();

    void setClientId(String str);

    void setControlName(String str);

    void setEventType(byte b);

    void setHeader(Header header);

    void setMarkerName(String str);

    void setMenuEntryId(int i);

    void setMousePoint(Point point);

    void setMousePointValid(boolean z);

    void setPose(Pose pose);
}
