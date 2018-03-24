using Homies.SARP.Kinematics.Enums;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Common
{
	public class RobotPoseStatusConfiguration
	{
		public RobotPoseStatus ForwardBackup { get; set; }
		public RobotPoseStatus Elbow { get; set; }
		public RobotPoseStatus Wrist { get; set; }

		public RobotPoseStatusConfiguration(RobotPoseStatus forwardBackup, RobotPoseStatus elbow, RobotPoseStatus wrist)
		{
			ForwardBackup = forwardBackup;
			Elbow = elbow;
			Wrist = wrist;

		}
	}
}
