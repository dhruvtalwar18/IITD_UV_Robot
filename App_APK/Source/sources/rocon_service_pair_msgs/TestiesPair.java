package rocon_service_pair_msgs;

import org.ros.internal.message.Message;

public interface TestiesPair extends Message {
    public static final String _DEFINITION = "\nTestiesPairRequest pair_request\nTestiesPairResponse pair_response\n";
    public static final String _TYPE = "rocon_service_pair_msgs/TestiesPair";

    TestiesPairRequest getPairRequest();

    TestiesPairResponse getPairResponse();

    void setPairRequest(TestiesPairRequest testiesPairRequest);

    void setPairResponse(TestiesPairResponse testiesPairResponse);
}
