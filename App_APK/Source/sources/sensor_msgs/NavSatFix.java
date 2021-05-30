package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface NavSatFix extends Message {
    public static final byte COVARIANCE_TYPE_APPROXIMATED = 1;
    public static final byte COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
    public static final byte COVARIANCE_TYPE_KNOWN = 3;
    public static final byte COVARIANCE_TYPE_UNKNOWN = 0;
    public static final String _DEFINITION = "# Navigation Satellite fix for any Global Navigation Satellite System\n#\n# Specified using the WGS 84 reference ellipsoid\n\n# header.stamp specifies the ROS time for this measurement (the\n#        corresponding satellite time may be reported using the\n#        sensor_msgs/TimeReference message).\n#\n# header.frame_id is the frame of reference reported by the satellite\n#        receiver, usually the location of the antenna.  This is a\n#        Euclidean frame relative to the vehicle, not a reference\n#        ellipsoid.\nHeader header\n\n# satellite fix status information\nNavSatStatus status\n\n# Latitude [degrees]. Positive is north of equator; negative is south.\nfloat64 latitude\n\n# Longitude [degrees]. Positive is east of prime meridian; negative is west.\nfloat64 longitude\n\n# Altitude [m]. Positive is above the WGS 84 ellipsoid\n# (quiet NaN if no altitude is available).\nfloat64 altitude\n\n# Position covariance [m^2] defined relative to a tangential plane\n# through the reported position. The components are East, North, and\n# Up (ENU), in row-major order.\n#\n# Beware: this coordinate system exhibits singularities at the poles.\n\nfloat64[9] position_covariance\n\n# If the covariance of the fix is known, fill it in completely. If the\n# GPS receiver provides the variance of each measurement, put them\n# along the diagonal. If only Dilution of Precision is available,\n# estimate an approximate covariance from that.\n\nuint8 COVARIANCE_TYPE_UNKNOWN = 0\nuint8 COVARIANCE_TYPE_APPROXIMATED = 1\nuint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\nuint8 COVARIANCE_TYPE_KNOWN = 3\n\nuint8 position_covariance_type\n";
    public static final String _TYPE = "sensor_msgs/NavSatFix";

    double getAltitude();

    Header getHeader();

    double getLatitude();

    double getLongitude();

    double[] getPositionCovariance();

    byte getPositionCovarianceType();

    NavSatStatus getStatus();

    void setAltitude(double d);

    void setHeader(Header header);

    void setLatitude(double d);

    void setLongitude(double d);

    void setPositionCovariance(double[] dArr);

    void setPositionCovarianceType(byte b);

    void setStatus(NavSatStatus navSatStatus);
}
