using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Homies.SARP.Kinematics.Base;
using Homies.SARP.Mathematics.Transformations;

namespace Homies.SARP.Kinematics.Forward
{
	public class SerialChainKinematics : IForwardKinematics
	{



		public TransformationMatrix GetTerminalFrame(List<Joint> joints)
		{
			var terminalFrame = new TransformationMatrix();


			foreach (var joint in joints)
			{

			}




			throw new NotImplementedException();
		}

		public TransformationMatrix GetTerminalFrame(List<Joint> joints, List<double> jointValues)
		{
			throw new NotImplementedException();
		}

		public int GetStatus(List<Joint> joints, List<double> jointValues)
		{
			throw new NotImplementedException();
		}

		public int GetTurn(List<Joint> joints, List<double> jointValues)
		{
			throw new NotImplementedException();
		}
	}
}
