using Homies.SARP.Kinematics.Base;
using Homies.SARP.Mathematics.Transformations;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Kinematics.Forward
{
	public interface IForwardKinematics
	{
		TransformationMatrix GetTerminalFrame(List<Joint> joints);
		TransformationMatrix GetTerminalFrame(List<Joint> joints, List<double> jointValues);
		int GetStatus(List<Joint> joints, List<double> jointValues);
		int GetTurn(List<Joint> joints, List<double> jointValues);
	}
}
