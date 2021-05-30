package rocon_app_manager_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface IncompatibleRappList extends Message {
    public static final String _DEFINITION = "# List all apps which were filtered for some reason or another\n\nstring[] blacklisted_rapps\nstring[] non_whitelisted_rapps\nstring[] platform_incompatible_rapps\nstring[] capabilities_incompatible_rapps\n";
    public static final String _TYPE = "rocon_app_manager_msgs/IncompatibleRappList";

    List<String> getBlacklistedRapps();

    List<String> getCapabilitiesIncompatibleRapps();

    List<String> getNonWhitelistedRapps();

    List<String> getPlatformIncompatibleRapps();

    void setBlacklistedRapps(List<String> list);

    void setCapabilitiesIncompatibleRapps(List<String> list);

    void setNonWhitelistedRapps(List<String> list);

    void setPlatformIncompatibleRapps(List<String> list);
}
