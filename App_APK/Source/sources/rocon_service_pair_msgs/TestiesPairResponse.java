package rocon_service_pair_msgs;

import org.ros.internal.message.Message;
import uuid_msgs.UniqueID;

public interface TestiesPairResponse extends Message {
    public static final String _DEFINITION = "uuid_msgs/UniqueID id\nTestiesResponse response\n";
    public static final String _TYPE = "rocon_service_pair_msgs/TestiesPairResponse";

    UniqueID getId();

    TestiesResponse getResponse();

    void setId(UniqueID uniqueID);

    void setResponse(TestiesResponse testiesResponse);
}
