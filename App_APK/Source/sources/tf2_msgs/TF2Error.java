package tf2_msgs;

import org.ros.internal.message.Message;

public interface TF2Error extends Message {
    public static final byte CONNECTIVITY_ERROR = 2;
    public static final byte EXTRAPOLATION_ERROR = 3;
    public static final byte INVALID_ARGUMENT_ERROR = 4;
    public static final byte LOOKUP_ERROR = 1;
    public static final byte NO_ERROR = 0;
    public static final byte TIMEOUT_ERROR = 5;
    public static final byte TRANSFORM_ERROR = 6;
    public static final String _DEFINITION = "uint8 NO_ERROR = 0\nuint8 LOOKUP_ERROR = 1\nuint8 CONNECTIVITY_ERROR = 2\nuint8 EXTRAPOLATION_ERROR = 3\nuint8 INVALID_ARGUMENT_ERROR = 4\nuint8 TIMEOUT_ERROR = 5\nuint8 TRANSFORM_ERROR = 6\n\nuint8 error\nstring error_string\n";
    public static final String _TYPE = "tf2_msgs/TF2Error";

    byte getError();

    String getErrorString();

    void setError(byte b);

    void setErrorString(String str);
}
