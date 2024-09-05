/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */



using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes.Moveit
{
    public class PlanningScene : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PlanningScene";

        //  name of planning scene
        public string name { get; set; }
        //  full robot state
        public RobotState robot_state { get; set; }
        //  The name of the robot model this scene is for
        public string robot_model_name { get; set; }
        // additional frames for duplicating tf (with respect to the planning frame)
        public TransformStamped[] fixed_frame_transforms { get; set; }
        // full allowed collision matrix
        public AllowedCollisionMatrix allowed_collision_matrix { get; set; }
        //  all link paddings
        public LinkPadding[] link_padding { get; set; }
        //  all link scales
        public LinkScale[] link_scale { get; set; }
        //  Attached objects, collision objects, even the octomap or collision map can have
        //  colors associated to them. This array specifies them.
        public ObjectColor[] object_colors { get; set; }
        //  the collision map
        public PlanningSceneWorld world { get; set; }
        //  Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
        public bool is_diff { get; set; }

        public PlanningScene()
        {
            this.name = "";
            this.robot_state = new RobotState();
            this.robot_model_name = "";
            this.fixed_frame_transforms = new TransformStamped[0];
            this.allowed_collision_matrix = new AllowedCollisionMatrix();
            this.link_padding = new LinkPadding[0];
            this.link_scale = new LinkScale[0];
            this.object_colors = new ObjectColor[0];
            this.world = new PlanningSceneWorld();
            this.is_diff = false;
        }

        public PlanningScene(string name, RobotState robot_state, string robot_model_name, TransformStamped[] fixed_frame_transforms, AllowedCollisionMatrix allowed_collision_matrix, LinkPadding[] link_padding, LinkScale[] link_scale, ObjectColor[] object_colors, PlanningSceneWorld world, bool is_diff)
        {
            this.name = name;
            this.robot_state = robot_state;
            this.robot_model_name = robot_model_name;
            this.fixed_frame_transforms = fixed_frame_transforms;
            this.allowed_collision_matrix = allowed_collision_matrix;
            this.link_padding = link_padding;
            this.link_scale = link_scale;
            this.object_colors = object_colors;
            this.world = world;
            this.is_diff = is_diff;
        }
    }
}
