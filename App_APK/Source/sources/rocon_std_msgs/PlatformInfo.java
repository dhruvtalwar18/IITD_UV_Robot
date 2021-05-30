package rocon_std_msgs;

import org.ros.internal.message.Message;

public interface PlatformInfo extends Message {
    public static final String _DEFINITION = "# Provides platform details for robots, software or human\n# interactive devices.\n\n########################### Variables ###########################\n\n# rocon universal resource identifier\nstring uri\n# rocon version compatibility identifier (used when connecting to concerts)\nstring version\nIcon icon\n";
    public static final String _TYPE = "rocon_std_msgs/PlatformInfo";

    Icon getIcon();

    String getUri();

    String getVersion();

    void setIcon(Icon icon);

    void setUri(String str);

    void setVersion(String str);
}
