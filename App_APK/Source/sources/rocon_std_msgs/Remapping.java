package rocon_std_msgs;

import org.ros.internal.message.Message;

public interface Remapping extends Message {
    public static final String _DEFINITION = "# Describes your typical ros remapping\n\nstring remap_from\nstring remap_to\n";
    public static final String _TYPE = "rocon_std_msgs/Remapping";

    String getRemapFrom();

    String getRemapTo();

    void setRemapFrom(String str);

    void setRemapTo(String str);
}
