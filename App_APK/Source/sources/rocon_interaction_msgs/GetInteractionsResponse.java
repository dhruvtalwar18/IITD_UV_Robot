package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface GetInteractionsResponse extends Message {
    public static final String _DEFINITION = "Interaction[] interactions";
    public static final String _TYPE = "rocon_interaction_msgs/GetInteractionsResponse";

    List<Interaction> getInteractions();

    void setInteractions(List<Interaction> list);
}
