package rocon_app_manager_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface GetRappListResponse extends Message {
    public static final String _DEFINITION = "Rapp[] available_rapps\nRapp[] running_rapps";
    public static final String _TYPE = "rocon_app_manager_msgs/GetRappListResponse";

    List<Rapp> getAvailableRapps();

    List<Rapp> getRunningRapps();

    void setAvailableRapps(List<Rapp> list);

    void setRunningRapps(List<Rapp> list);
}
