package geometry_msgs;

import org.ros.internal.message.Message;

public interface Transform extends Message {
    public static final String _DEFINITION = "# This represents the transform between two coordinate frames in free space.\n\nVector3 translation\nQuaternion rotation\n";
    public static final String _TYPE = "geometry_msgs/Transform";

    Quaternion getRotation();

    Vector3 getTranslation();

    void setRotation(Quaternion quaternion);

    void setTranslation(Vector3 vector3);
}
