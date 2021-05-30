package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface InertiaStamped extends Message {
    public static final String _DEFINITION = "Header header\nInertia inertia\n";
    public static final String _TYPE = "geometry_msgs/InertiaStamped";

    Header getHeader();

    Inertia getInertia();

    void setHeader(Header header);

    void setInertia(Inertia inertia);
}
