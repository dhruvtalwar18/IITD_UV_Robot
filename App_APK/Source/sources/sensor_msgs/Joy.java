package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Joy extends Message {
    public static final String _DEFINITION = "# Reports the state of a joysticks axes and buttons.\nHeader header           # timestamp in the header is the time the data is received from the joystick\nfloat32[] axes          # the axes measurements from a joystick\nint32[] buttons         # the buttons measurements from a joystick \n";
    public static final String _TYPE = "sensor_msgs/Joy";

    float[] getAxes();

    int[] getButtons();

    Header getHeader();

    void setAxes(float[] fArr);

    void setButtons(int[] iArr);

    void setHeader(Header header);
}
