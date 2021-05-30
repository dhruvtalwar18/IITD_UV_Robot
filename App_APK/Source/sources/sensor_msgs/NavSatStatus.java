package sensor_msgs;

import org.ros.internal.message.Message;

public interface NavSatStatus extends Message {
    public static final short SERVICE_COMPASS = 4;
    public static final short SERVICE_GALILEO = 8;
    public static final short SERVICE_GLONASS = 2;
    public static final short SERVICE_GPS = 1;
    public static final byte STATUS_FIX = 0;
    public static final byte STATUS_GBAS_FIX = 2;
    public static final byte STATUS_NO_FIX = -1;
    public static final byte STATUS_SBAS_FIX = 1;
    public static final String _DEFINITION = "# Navigation Satellite fix status for any Global Navigation Satellite System\n\n# Whether to output an augmented fix is determined by both the fix\n# type and the last time differential corrections were received.  A\n# fix is valid when status >= STATUS_FIX.\n\nint8 STATUS_NO_FIX =  -1        # unable to fix position\nint8 STATUS_FIX =      0        # unaugmented fix\nint8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation\nint8 STATUS_GBAS_FIX = 2        # with ground-based augmentation\n\nint8 status\n\n# Bits defining which Global Navigation Satellite System signals were\n# used by the receiver.\n\nuint16 SERVICE_GPS =     1\nuint16 SERVICE_GLONASS = 2\nuint16 SERVICE_COMPASS = 4      # includes BeiDou.\nuint16 SERVICE_GALILEO = 8\n\nuint16 service\n";
    public static final String _TYPE = "sensor_msgs/NavSatStatus";

    short getService();

    byte getStatus();

    void setService(short s);

    void setStatus(byte b);
}
