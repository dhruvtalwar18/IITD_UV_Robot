package rocon_app_manager_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import rocon_std_msgs.Icon;
import rocon_std_msgs.KeyValue;

public interface Rapp extends Message {
    public static final String _DEFINITION = "# This is the message that gets published in list_rapps. Do not think of it as the\n# 'rapp' definition (since that is quite varied -> ancestor, virtual, child, implementation)\n# Rather it is the published list of rapps with the required information for the concert\n# and some introspection.\n\n# app name (ros resource name format, i.e. pkg/name, e.g. turtle_concert/teleop)\nstring name\n# user-friendly display name\nstring display_name\nstring description\n# a rocon uri string indicating platform compatibility\nstring compatibility\nstring status\n\n# a list of implementations\nstring[] implementations\n\n# A preferred rapp for virtual rapp\nstring preferred\n\n# icon for showing the app\nrocon_std_msgs/Icon icon\n\n# public interface and parameters\nrocon_std_msgs/KeyValue[] public_interface\nrocon_std_msgs/KeyValue[] public_parameters\n";
    public static final String _TYPE = "rocon_app_manager_msgs/Rapp";

    String getCompatibility();

    String getDescription();

    String getDisplayName();

    Icon getIcon();

    List<String> getImplementations();

    String getName();

    String getPreferred();

    List<KeyValue> getPublicInterface();

    List<KeyValue> getPublicParameters();

    String getStatus();

    void setCompatibility(String str);

    void setDescription(String str);

    void setDisplayName(String str);

    void setIcon(Icon icon);

    void setImplementations(List<String> list);

    void setName(String str);

    void setPreferred(String str);

    void setPublicInterface(List<KeyValue> list);

    void setPublicParameters(List<KeyValue> list);

    void setStatus(String str);
}
