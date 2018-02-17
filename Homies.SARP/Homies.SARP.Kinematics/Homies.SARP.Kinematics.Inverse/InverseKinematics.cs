using System;
using System.Collections.Generic;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Diagnostics;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Inverse
{
	public class InverseKinematics
	{
		public List<double> ResultAxisValues { get; set; }

		public List<double> GetInverseKinematics(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
		{
			List<double> returnAngles = new List<double>();

			TransformationMatrix wrist = new TransformationMatrix(
				targetMatrix.DenseMatrix * 
				(DenseMatrix)dhParam.Last().JointTransform.DenseMatrix.Inverse());

			double angleDeg1Plus = Math.Atan2(wrist.Matrix3D.OffsetY, wrist.Matrix3D.OffsetX) * 180 / Math.PI;
			double angleDeg1Minus = Math.Atan2(-wrist.Matrix3D.OffsetY, -wrist.Matrix3D.OffsetX) * 180 / Math.PI;

			returnAngles.Add(angleDeg1Plus);
			returnAngles.Add(angleDeg1Minus);

			return returnAngles;
		}
	}
}
