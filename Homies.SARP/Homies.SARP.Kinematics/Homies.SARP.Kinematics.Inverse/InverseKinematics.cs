using System;
using System.Collections.Generic;
using Homies.SARP.Mathematics.Transformations;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Inverse
{
	public class InverseKinematics
	{
		TransformationMatrix _targetFrame;
		TransformationMatrix _wristFrame;

		public List<double> ResultAxisValues { get; set; }

		public List<double> GetInverseKinematics(TransformationMatrix targetMatrix)
		{
			throw new NotImplementedException();
		}
	}
}
