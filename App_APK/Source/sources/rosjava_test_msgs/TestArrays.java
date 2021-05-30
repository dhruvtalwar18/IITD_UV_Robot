package rosjava_test_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import org.ros.message.Time;

public interface TestArrays extends Message {
    public static final String _DEFINITION = "# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\n\nint32[] int32_array\nfloat32[] float32_array\ntime[] time_array\nTestString[] test_string_array\n# TODO: array of arrays\n";
    public static final String _TYPE = "rosjava_test_msgs/TestArrays";

    String getCallerId();

    float[] getFloat32Array();

    int[] getInt32Array();

    String getOrigCallerId();

    List<TestString> getTestStringArray();

    List<Time> getTimeArray();

    void setCallerId(String str);

    void setFloat32Array(float[] fArr);

    void setInt32Array(int[] iArr);

    void setOrigCallerId(String str);

    void setTestStringArray(List<TestString> list);

    void setTimeArray(List<Time> list);
}
