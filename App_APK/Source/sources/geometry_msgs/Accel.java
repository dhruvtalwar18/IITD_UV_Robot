package geometry_msgs;

import org.ros.internal.message.Message;

public interface Accel extends Message {
    public static final String _DEFINITION = "# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n";
    public static final String _TYPE = "geometry_msgs/Accel";

    Vector3 getAngular();

    Vector3 getLinear();

    void setAngular(Vector3 vector3);

    void setLinear(Vector3 vector3);
}
