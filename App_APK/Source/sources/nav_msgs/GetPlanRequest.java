package nav_msgs;

import geometry_msgs.PoseStamped;
import org.ros.internal.message.Message;

public interface GetPlanRequest extends Message {
    public static final String _DEFINITION = "# Get a plan from the current position to the goal Pose \n\n# The start pose for the plan\ngeometry_msgs/PoseStamped start\n\n# The final pose of the goal position\ngeometry_msgs/PoseStamped goal\n\n# If the goal is obstructed, how many meters the planner can \n# relax the constraint in x and y before failing. \nfloat32 tolerance\n";
    public static final String _TYPE = "nav_msgs/GetPlanRequest";

    PoseStamped getGoal();

    PoseStamped getStart();

    float getTolerance();

    void setGoal(PoseStamped poseStamped);

    void setStart(PoseStamped poseStamped);

    void setTolerance(float f);
}
