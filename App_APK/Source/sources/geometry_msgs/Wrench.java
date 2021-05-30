package geometry_msgs;

import org.ros.internal.message.Message;

public interface Wrench extends Message {
    public static final String _DEFINITION = "# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n";
    public static final String _TYPE = "geometry_msgs/Wrench";

    Vector3 getForce();

    Vector3 getTorque();

    void setForce(Vector3 vector3);

    void setTorque(Vector3 vector3);
}
