using System;
using System.Collections.Generic;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Diagnostics;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Inverse
{

	//TODO: 
	// 1. Checking if target frame is in workspace
	// 2. Checking when multiple angles (solutions) which of the possible solutions is closer / possible
	// 3. When turn / status is specified choose correct angle
	public class InverseKinematics
	{
		public List<double> ResultAxisValues { get; set; }
		
		public List<double> GetInverseKinematics(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
		{
			List<double> returnAngles = new List<double>();

			TransformationMatrix wristToAxis1 = new TransformationMatrix(
				(DenseMatrix)dhParam.First().JointStandardTransform.DenseMatrix.Inverse() * 
				targetMatrix.DenseMatrix * 
				(DenseMatrix)dhParam.Last().JointTransform.DenseMatrix.Inverse());

			double angleDeg1Plus = Math.Atan2(wristToAxis1.Matrix3D.OffsetY, wristToAxis1.Matrix3D.OffsetX) * 180 / Math.PI;
			double angleDeg1Minus = Math.Atan2(-wristToAxis1.Matrix3D.OffsetY, -wristToAxis1.Matrix3D.OffsetX) * 180 / Math.PI;

			

			returnAngles.Add(angleDeg1Plus);
			returnAngles.Add(angleDeg1Minus);

			return returnAngles;
		}
	}
}
