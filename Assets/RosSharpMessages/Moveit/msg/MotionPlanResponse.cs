/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */



namespace RosSharp.RosBridgeClient.MessageTypes.Moveit
{
    public class MotionPlanResponse : Message
    {
        public const string RosMessageName = "moveit_msgs-master/MotionPlanResponse";

        //  The representation of a solution to a planning problem
        //  The corresponding robot state
        public RobotState trajectory_start { get; set; }
        //  The group used for planning (usually the same as in the request)
        public string group_name { get; set; }
        //  A solution trajectory, if found
        public RobotTrajectory trajectory { get; set; }
        //  Planning time (seconds)
        public double planning_time { get; set; }
        //  Error code - encodes the overall reason for failure
        public MoveItErrorCodes error_code { get; set; }

        public MotionPlanResponse()
        {
            this.trajectory_start = new RobotState();
            this.group_name = "";
            this.trajectory = new RobotTrajectory();
            this.planning_time = 0.0;
            this.error_code = new MoveItErrorCodes();
        }

        public MotionPlanResponse(RobotState trajectory_start, string group_name, RobotTrajectory trajectory, double planning_time, MoveItErrorCodes error_code)
        {
            this.trajectory_start = trajectory_start;
            this.group_name = group_name;
            this.trajectory = trajectory;
            this.planning_time = planning_time;
            this.error_code = error_code;
        }
    }
}
