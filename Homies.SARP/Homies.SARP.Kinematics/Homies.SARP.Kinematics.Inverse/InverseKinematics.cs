using Homies.SARP.Mathematics.Transformations;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Inverse
{
	public abstract class InverseKinematics
	{
		public List<double> ResultAxisValues { get; set; }

		List<double> GetInverseKinematics(TransformationMatrix targetMatrix)
		{
			throw new NotImplementedException();
		}
	}
}
