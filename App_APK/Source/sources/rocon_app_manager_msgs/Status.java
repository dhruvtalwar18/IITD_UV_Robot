package rocon_app_manager_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import rocon_std_msgs.KeyValue;

public interface Status extends Message {
    public static final String RAPP_RUNNING = "running";
    public static final String RAPP_STOPPED = "stopped";
    public static final String _DEFINITION = "# Namespace under which applications will run if connected\nstring application_namespace\n\n# Who is controlling the app manager (i.e. who invited it to send it's\n# control handles). If the empty string, it is not being controlled \n# and subsequently is available\nstring remote_controller\n\n# Rapp is running or not\nstring RAPP_STOPPED=stopped\nstring RAPP_RUNNING=running\nstring rapp_status\n\n# Current app details (if running), a default Rapp() (filled with empty strings and lists) otherwise\nRapp rapp\n\n# The runtime, possibly namespaced and remapped interfaces/parameters\nPublishedInterface[] published_interfaces\nrocon_std_msgs/KeyValue[] published_parameters\n";
    public static final String _TYPE = "rocon_app_manager_msgs/Status";

    String getApplicationNamespace();

    List<PublishedInterface> getPublishedInterfaces();

    List<KeyValue> getPublishedParameters();

    Rapp getRapp();

    String getRappStatus();

    String getRemoteController();

    void setApplicationNamespace(String str);

    void setPublishedInterfaces(List<PublishedInterface> list);

    void setPublishedParameters(List<KeyValue> list);

    void setRapp(Rapp rapp);

    void setRappStatus(String str);

    void setRemoteController(String str);
}
