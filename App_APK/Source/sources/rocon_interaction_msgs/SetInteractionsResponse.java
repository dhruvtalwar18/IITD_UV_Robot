package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface SetInteractionsResponse extends Message {
    public static final String _DEFINITION = "\nbool result\n\n# Could use better error handling here - provide a list of what got\n# got manipulated and what did not.";
    public static final String _TYPE = "rocon_interaction_msgs/SetInteractionsResponse";

    boolean getResult();

    void setResult(boolean z);
}
