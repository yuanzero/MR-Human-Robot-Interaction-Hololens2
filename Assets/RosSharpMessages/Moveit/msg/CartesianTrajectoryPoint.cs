/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */



using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient.MessageTypes.Moveit
{
    public class CartesianTrajectoryPoint : Message
    {
        public const string RosMessageName = "moveit_msgs-master/CartesianTrajectoryPoint";

        //  The definition of a cartesian point in a trajectory. Defines the cartesian state of the point and it's time,
        //  following the pattern of the JointTrajectory message
        public CartesianPoint point { get; set; }
        public Duration time_from_start { get; set; }

        public CartesianTrajectoryPoint()
        {
            this.point = new CartesianPoint();
            this.time_from_start = new Duration();
        }

        public CartesianTrajectoryPoint(CartesianPoint point, Duration time_from_start)
        {
            this.point = point;
            this.time_from_start = time_from_start;
        }
    }
}
