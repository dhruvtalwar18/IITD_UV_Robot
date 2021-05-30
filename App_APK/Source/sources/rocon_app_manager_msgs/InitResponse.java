package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface InitResponse extends Message {
    public static final String _DEFINITION = "bool result";
    public static final String _TYPE = "rocon_app_manager_msgs/InitResponse";

    boolean getResult();

    void setResult(boolean z);
}
