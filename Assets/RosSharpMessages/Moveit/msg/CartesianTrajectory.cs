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
    public class CartesianTrajectory : Message
    {
        public const string RosMessageName = "moveit_msgs-master/CartesianTrajectory";

        //  This message describes the trajectory of a tracked frame in task-space
        public Header header { get; set; }
        //  The name of the Cartesian frame being tracked with respect to the base frame provided in header.frame_id
        public string tracked_frame { get; set; }
        public CartesianTrajectoryPoint[] points { get; set; }

        public CartesianTrajectory()
        {
            this.header = new Header();
            this.tracked_frame = "";
            this.points = new CartesianTrajectoryPoint[0];
        }

        public CartesianTrajectory(Header header, string tracked_frame, CartesianTrajectoryPoint[] points)
        {
            this.header = header;
            this.tracked_frame = tracked_frame;
            this.points = points;
        }
    }
}
