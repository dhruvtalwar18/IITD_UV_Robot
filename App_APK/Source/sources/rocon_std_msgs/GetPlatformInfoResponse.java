package rocon_std_msgs;

import org.ros.internal.message.Message;

public interface GetPlatformInfoResponse extends Message {
    public static final String _DEFINITION = "PlatformInfo platform_info";
    public static final String _TYPE = "rocon_std_msgs/GetPlatformInfoResponse";

    PlatformInfo getPlatformInfo();

    void setPlatformInfo(PlatformInfo platformInfo);
}
