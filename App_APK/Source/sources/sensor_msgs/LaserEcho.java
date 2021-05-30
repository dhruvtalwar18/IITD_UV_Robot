package sensor_msgs;

import org.ros.internal.message.Message;

public interface LaserEcho extends Message {
    public static final String _DEFINITION = "# This message is a submessage of MultiEchoLaserScan and is not intended\n# to be used separately.\n\nfloat32[] echoes  # Multiple values of ranges or intensities.\n                  # Each array represents data from the same angle increment.";
    public static final String _TYPE = "sensor_msgs/LaserEcho";

    float[] getEchoes();

    void setEchoes(float[] fArr);
}
