package diagnostic_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface DiagnosticStatus extends Message {
    public static final byte ERROR = 2;
    public static final byte OK = 0;
    public static final byte STALE = 3;
    public static final byte WARN = 1;
    public static final String _DEFINITION = "# This message holds the status of an individual component of the robot.\n# \n\n# Possible levels of operations\nbyte OK=0\nbyte WARN=1\nbyte ERROR=2\nbyte STALE=3\n\nbyte level # level of operation enumerated above \nstring name # a description of the test/component reporting\nstring message # a description of the status\nstring hardware_id # a hardware unique string\nKeyValue[] values # an array of values associated with the status\n\n";
    public static final String _TYPE = "diagnostic_msgs/DiagnosticStatus";

    String getHardwareId();

    byte getLevel();

    String getMessage();

    String getName();

    List<KeyValue> getValues();

    void setHardwareId(String str);

    void setLevel(byte b);

    void setMessage(String str);

    void setName(String str);

    void setValues(List<KeyValue> list);
}
