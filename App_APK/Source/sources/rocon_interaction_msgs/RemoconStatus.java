package rocon_interaction_msgs;

import org.ros.internal.message.Message;
import rocon_std_msgs.PlatformInfo;

public interface RemoconStatus extends Message {
    public static final String _DEFINITION = "# Used by the remocons to inform the concert of it's current status. They should\n# publish this as a latched publisher.\n\nrocon_std_msgs/PlatformInfo platform_info\n# The remocon id\n# This should be a '32 character Type 4 uuid hex string'\nstring uuid\n\n\n# We should be using this, but java can't handle the type, reintegrate when we bugfix.\n# uuid_msgs/UniqueID[] running_interactions\n\n# This is a crc32 hash code we use because of the above.\nint32[] running_interactions\n\n# rocon version compatibility identifier (used when connecting to concerts)\nstring version\n";
    public static final String _TYPE = "rocon_interaction_msgs/RemoconStatus";

    PlatformInfo getPlatformInfo();

    int[] getRunningInteractions();

    String getUuid();

    String getVersion();

    void setPlatformInfo(PlatformInfo platformInfo);

    void setRunningInteractions(int[] iArr);

    void setUuid(String str);

    void setVersion(String str);
}
