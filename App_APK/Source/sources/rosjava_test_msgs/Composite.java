package rosjava_test_msgs;

import org.ros.internal.message.Message;

public interface Composite extends Message {
    public static final String _DEFINITION = "# composite message. required for testing import calculation in generators\nCompositeA a\nCompositeB b\n";
    public static final String _TYPE = "rosjava_test_msgs/Composite";

    CompositeA getA();

    CompositeB getB();

    void setA(CompositeA compositeA);

    void setB(CompositeB compositeB);
}
