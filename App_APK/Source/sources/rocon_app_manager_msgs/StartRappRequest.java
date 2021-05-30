package rocon_app_manager_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import rocon_std_msgs.KeyValue;
import rocon_std_msgs.Remapping;

public interface StartRappRequest extends Message {
    public static final String _DEFINITION = "# Name of the rapp to launch\nstring name\nrocon_std_msgs/Remapping[] remappings\n\n# Key value pairs representing rapp parameters\nrocon_std_msgs/KeyValue[] parameters\n";
    public static final String _TYPE = "rocon_app_manager_msgs/StartRappRequest";

    String getName();

    List<KeyValue> getParameters();

    List<Remapping> getRemappings();

    void setName(String str);

    void setParameters(List<KeyValue> list);

    void setRemappings(List<Remapping> list);
}
