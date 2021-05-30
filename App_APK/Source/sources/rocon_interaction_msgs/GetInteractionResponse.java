package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface GetInteractionResponse extends Message {
    public static final String _DEFINITION = "\nbool result\nInteraction interaction";
    public static final String _TYPE = "rocon_interaction_msgs/GetInteractionResponse";

    Interaction getInteraction();

    boolean getResult();

    void setInteraction(Interaction interaction);

    void setResult(boolean z);
}
