package visualization_msgs;

import geometry_msgs.Pose;
import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface InteractiveMarker extends Message {
    public static final String _DEFINITION = "# Time/frame info.\n# If header.time is set to 0, the marker will be retransformed into\n# its frame on each timestep. You will receive the pose feedback\n# in the same frame.\n# Otherwise, you might receive feedback in a different frame.\n# For rviz, this will be the current 'fixed frame' set by the user.\nHeader header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n\n# Short description (< 40 characters).\nstring description\n\n# Scale to be used for default controls (default=1).\nfloat32 scale\n\n# All menu and submenu entries associated with this marker.\nMenuEntry[] menu_entries\n\n# List of controls displayed for this marker.\nInteractiveMarkerControl[] controls\n";
    public static final String _TYPE = "visualization_msgs/InteractiveMarker";

    List<InteractiveMarkerControl> getControls();

    String getDescription();

    Header getHeader();

    List<MenuEntry> getMenuEntries();

    String getName();

    Pose getPose();

    float getScale();

    void setControls(List<InteractiveMarkerControl> list);

    void setDescription(String str);

    void setHeader(Header header);

    void setMenuEntries(List<MenuEntry> list);

    void setName(String str);

    void setPose(Pose pose);

    void setScale(float f);
}
