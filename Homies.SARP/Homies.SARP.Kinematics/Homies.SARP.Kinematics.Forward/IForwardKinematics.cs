using Homies.SARP.Kinematics.Common;
using Homies.SARP.Mathematics.Transformations;
using System.Collections.Generic;

namespace Homies.SARP.Kinematics.Forward
{
	public interface IForwardKinematics
	{
		TransformationMatrix GetTerminalFrame(List<DHParameter> joints);
		TransformationMatrix GetTerminalFrame(List<DHParameter> joints, List<double> jointValues);
		int GetStatus(List<DHParameter> joints, List<double> jointValues);
		int GetTurn(List<DHParameter> joints, List<double> jointValues);
	}
}
