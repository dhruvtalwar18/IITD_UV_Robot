package rocon_service_pair_msgs;

import org.ros.internal.message.Message;
import uuid_msgs.UniqueID;

public interface TestiesPairRequest extends Message {
    public static final String _DEFINITION = "uuid_msgs/UniqueID id\nTestiesRequest request\n";
    public static final String _TYPE = "rocon_service_pair_msgs/TestiesPairRequest";

    UniqueID getId();

    TestiesRequest getRequest();

    void setId(UniqueID uniqueID);

    void setRequest(TestiesRequest testiesRequest);
}
