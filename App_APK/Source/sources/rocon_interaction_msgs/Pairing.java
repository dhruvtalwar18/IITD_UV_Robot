package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import rocon_std_msgs.KeyValue;
import rocon_std_msgs.Remapping;

public interface Pairing extends Message {
    public static final String _DEFINITION = "# Properties of a paired app that is destined to work with a rocon interaction.\n\n# ros resource name of the rapp, e.g. rocon_apps/talker\nstring rapp\n\nrocon_std_msgs/Remapping[] remappings\n\n# Key value pairs representing rapp parameters\nrocon_std_msgs/KeyValue[] parameters\n";
    public static final String _TYPE = "rocon_interaction_msgs/Pairing";

    List<KeyValue> getParameters();

    String getRapp();

    List<Remapping> getRemappings();

    void setParameters(List<KeyValue> list);

    void setRapp(String str);

    void setRemappings(List<Remapping> list);
}
