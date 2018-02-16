using Homies.SARP.Machines.MachineStructures;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Inverse
{
	public class InverseKinematics
	{
		TransformationMatrix _targetFrame;
		TransformationMatrix _wristFrame;
		Robot _robotModel;

		public List<double> ResultAxisValues { get; set; }

		public List<double> GetInverseKinematics(TransformationMatrix targetMatrix, Robot robiModel)
		{
			_robotModel = robiModel;
			_targetFrame = targetMatrix;
			_wristFrame = new TransformationMatrix(targetMatrix.DenseMatrix * (DenseMatrix)robiModel.Joints.Last().Value.JointTransformation.DenseMatrix.Inverse());

			throw new NotImplementedException();
		}
	}
}
