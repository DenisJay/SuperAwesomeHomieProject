using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra.Double;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;


namespace Homies.SARP.Kinematics.Forward
{
    public class RobotKinematics : Kinematics
    {
        public RobotKinematics(IReadOnlyCollection<DHParameter> dhParameter) : base(dhParameter)
        {

        }

		public override TransformationMatrix GetTerminalFrame(List<DHParameter> dhParameter)
		{
			throw new NotImplementedException();
		}

		public override TransformationMatrix GetTerminalFrame(List<DHParameter> dhParameter, List<double> jointValues)
		{
			throw new NotImplementedException();
		}

		public override int GetStatus(List<DHParameter> dhParameter, List<double> jointValues)
		{
			throw new NotImplementedException();
		}

		public override int GetTurn(List<DHParameter> dhParameter, List<double> jointValues)
		{
			throw new NotImplementedException();
		}
	}
}
