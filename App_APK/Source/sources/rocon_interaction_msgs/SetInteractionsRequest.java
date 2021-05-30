package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface SetInteractionsRequest extends Message {
    public static final String _DEFINITION = "# Extend or delete the rocon interactions database with this list. \n\nInteraction[] interactions\n\n# If true, load, else attempt to unload them.\nbool load\n\n";
    public static final String _TYPE = "rocon_interaction_msgs/SetInteractionsRequest";

    List<Interaction> getInteractions();

    boolean getLoad();

    void setInteractions(List<Interaction> list);

    void setLoad(boolean z);
}
