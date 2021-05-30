package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface LaserScan extends Message {
    public static final String _DEFINITION = "# Single scan from a planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nHeader header            # timestamp in the header is the acquisition time of \n                         # the first ray in the scan.\n                         #\n                         # in frame frame_id, angles are measured around \n                         # the positive Z axis (counterclockwise, if Z is up)\n                         # with zero angle being forward along the x axis\n                         \nfloat32 angle_min        # start angle of the scan [rad]\nfloat32 angle_max        # end angle of the scan [rad]\nfloat32 angle_increment  # angular distance between measurements [rad]\n\nfloat32 time_increment   # time between measurements [seconds] - if your scanner\n                         # is moving, this will be used in interpolating position\n                         # of 3d points\nfloat32 scan_time        # time between scans [seconds]\n\nfloat32 range_min        # minimum range value [m]\nfloat32 range_max        # maximum range value [m]\n\nfloat32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)\nfloat32[] intensities    # intensity data [device-specific units].  If your\n                         # device does not provide intensities, please leave\n                         # the array empty.\n";
    public static final String _TYPE = "sensor_msgs/LaserScan";

    float getAngleIncrement();

    float getAngleMax();

    float getAngleMin();

    Header getHeader();

    float[] getIntensities();

    float getRangeMax();

    float getRangeMin();

    float[] getRanges();

    float getScanTime();

    float getTimeIncrement();

    void setAngleIncrement(float f);

    void setAngleMax(float f);

    void setAngleMin(float f);

    void setHeader(Header header);

    void setIntensities(float[] fArr);

    void setRangeMax(float f);

    void setRangeMin(float f);

    void setRanges(float[] fArr);

    void setScanTime(float f);

    void setTimeIncrement(float f);
}
