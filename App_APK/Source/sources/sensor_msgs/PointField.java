package sensor_msgs;

import org.ros.internal.message.Message;

public interface PointField extends Message {
    public static final byte FLOAT32 = 7;
    public static final byte FLOAT64 = 8;
    public static final byte INT16 = 3;
    public static final byte INT32 = 5;
    public static final byte INT8 = 1;
    public static final byte UINT16 = 4;
    public static final byte UINT32 = 6;
    public static final byte UINT8 = 2;
    public static final String _DEFINITION = "# This message holds the description of one point entry in the\n# PointCloud2 message format.\nuint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\n\nstring name      # Name of field\nuint32 offset    # Offset from start of point struct\nuint8  datatype  # Datatype enumeration, see above\nuint32 count     # How many elements in the field\n";
    public static final String _TYPE = "sensor_msgs/PointField";

    int getCount();

    byte getDatatype();

    String getName();

    int getOffset();

    void setCount(int i);

    void setDatatype(byte b);

    void setName(String str);

    void setOffset(int i);
}
