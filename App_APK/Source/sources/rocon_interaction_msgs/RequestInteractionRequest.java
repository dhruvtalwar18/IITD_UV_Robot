package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface RequestInteractionRequest extends Message {
    public static final String _DEFINITION = "# This is used between remocons and the interactions manager to request\n# the launch of a remote application to interact with the concert.\n\n# Name of the remocon doing the requesting\nstring remocon\n\n# Hash id of the interaction being requested\nint32 hash\n\n";
    public static final String _TYPE = "rocon_interaction_msgs/RequestInteractionRequest";

    int getHash();

    String getRemocon();

    void setHash(int i);

    void setRemocon(String str);
}
