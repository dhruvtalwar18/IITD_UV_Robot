package std_msgs;

import org.ros.internal.message.Message;

public interface ColorRGBA extends Message {
    public static final String _DEFINITION = "float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n";
    public static final String _TYPE = "std_msgs/ColorRGBA";

    float getA();

    float getB();

    float getG();

    float getR();

    void setA(float f);

    void setB(float f);

    void setG(float f);

    void setR(float f);
}
