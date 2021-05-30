package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface SetInteractions extends Message {
    public static final String _DEFINITION = "# Extend or delete the rocon interactions database with this list. \n\nInteraction[] interactions\n\n# If true, load, else attempt to unload them.\nbool load\n\n---\n\nbool result\n\n# Could use better error handling here - provide a list of what got\n# got manipulated and what did not.\n\n";
    public static final String _TYPE = "rocon_interaction_msgs/SetInteractions";
}
