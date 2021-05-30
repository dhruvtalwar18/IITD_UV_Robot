package sensor_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface MultiEchoLaserScan extends Message {
    public static final String _DEFINITION = "# Single scan from a multi-echo planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nHeader header            # timestamp in the header is the acquisition time of \n                         # the first ray in the scan.\n                         #\n                         # in frame frame_id, angles are measured around \n                         # the positive Z axis (counterclockwise, if Z is up)\n                         # with zero angle being forward along the x axis\n                         \nfloat32 angle_min        # start angle of the scan [rad]\nfloat32 angle_max        # end angle of the scan [rad]\nfloat32 angle_increment  # angular distance between measurements [rad]\n\nfloat32 time_increment   # time between measurements [seconds] - if your scanner\n                         # is moving, this will be used in interpolating position\n                         # of 3d points\nfloat32 scan_time        # time between scans [seconds]\n\nfloat32 range_min        # minimum range value [m]\nfloat32 range_max        # maximum range value [m]\n\nLaserEcho[] ranges       # range data [m] (Note: NaNs, values < range_min or > range_max should be discarded)\n                         # +Inf measurements are out of range\n                         # -Inf measurements are too close to determine exact distance.\nLaserEcho[] intensities  # intensity data [device-specific units].  If your\n                         # device does not provide intensities, please leave\n                         # the array empty.";
    public static final String _TYPE = "sensor_msgs/MultiEchoLaserScan";

    float getAngleIncrement();

    float getAngleMax();

    float getAngleMin();

    Header getHeader();

    List<LaserEcho> getIntensities();

    float getRangeMax();

    float getRangeMin();

    List<LaserEcho> getRanges();

    float getScanTime();

    float getTimeIncrement();

    void setAngleIncrement(float f);

    void setAngleMax(float f);

    void setAngleMin(float f);

    void setHeader(Header header);

    void setIntensities(List<LaserEcho> list);

    void setRangeMax(float f);

    void setRangeMin(float f);

    void setRanges(List<LaserEcho> list);

    void setScanTime(float f);

    void setTimeIncrement(float f);
}
