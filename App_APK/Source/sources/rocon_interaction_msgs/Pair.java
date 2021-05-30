package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface Pair extends Message {
    public static final String _DEFINITION = "# Indicates what pairs (rapp, remocon) are running at any point in time.\n# If none, these values drop back to empty strings.\n\nstring rapp\nstring remocon\n";
    public static final String _TYPE = "rocon_interaction_msgs/Pair";

    String getRapp();

    String getRemocon();

    void setRapp(String str);

    void setRemocon(String str);
}
