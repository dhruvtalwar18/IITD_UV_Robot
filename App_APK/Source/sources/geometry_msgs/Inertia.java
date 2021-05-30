package geometry_msgs;

import org.ros.internal.message.Message;

public interface Inertia extends Message {
    public static final String _DEFINITION = "# Mass [kg]\nfloat64 m\n\n# Center of mass [m]\ngeometry_msgs/Vector3 com\n\n# Inertia Tensor [kg-m^2]\n#     | ixx ixy ixz |\n# I = | ixy iyy iyz |\n#     | ixz iyz izz |\nfloat64 ixx\nfloat64 ixy\nfloat64 ixz\nfloat64 iyy\nfloat64 iyz\nfloat64 izz\n";
    public static final String _TYPE = "geometry_msgs/Inertia";

    Vector3 getCom();

    double getIxx();

    double getIxy();

    double getIxz();

    double getIyy();

    double getIyz();

    double getIzz();

    double getM();

    void setCom(Vector3 vector3);

    void setIxx(double d);

    void setIxy(double d);

    void setIxz(double d);

    void setIyy(double d);

    void setIyz(double d);

    void setIzz(double d);

    void setM(double d);
}
