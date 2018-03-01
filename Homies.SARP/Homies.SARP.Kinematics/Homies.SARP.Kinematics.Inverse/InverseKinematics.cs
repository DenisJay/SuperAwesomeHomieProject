using System;
using System.Collections.Generic;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Diagnostics;

namespace Homies.SARP.Kinematics.Inverse
{

	//TODO: 
	// 1. Checking if target frame is in workspace
	// 2. Checking when multiple angles (solutions) which of the possible solutions is closer / possible
	// 3. When turn / status is specified choose correct angle
	public class InverseKinematics
	{

		public List<double[]> ResultAxisSolutions { get; private set; }
		public List<double> ResultAxisValues { get; private set; }

		public InverseKinematics()
		{
			ResultAxisSolutions = new List<double[]>();
			ResultAxisValues = new List<double>();
		}

		public List<double> GetAxisValues(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
		{
			List<double> returnAnglesForTest = new List<double>();

			ComputeAngle1Solutions(targetMatrix, dhParam);
			//ComputeAngle23Solutions(targetMatrix, dhParam);

			returnAnglesForTest.Add(ResultAxisSolutions[0][0]);
			

			//TODO: Implement "ChooseValidConfiguration()"
			//ChooseValidConfiguration()

			return returnAnglesForTest;
		}

		/// <summary>
		/// Very easy. look at the wrist point in the xy plane starting from standard axis 1 transformation
		/// (theta1 angle is neglected in order to get the sign correct. 
		/// Interesting for kuka since they have this weird convention where the rotation direction vector of the axis 1 is pointing downwards)
		/// </summary>
		/// <param name="targetMatrix">The target frame in some workspace</param>
		/// <param name="dhParam">the dhparameter of the kinematic chain</param>
		private void ComputeAngle1Solutions(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
		{
			TransformationMatrix wristToAxis1 = new TransformationMatrix( 
				(DenseMatrix)dhParam.First().JointStandardTransform.DenseMatrix.Inverse() * 
				targetMatrix.DenseMatrix * 
				(DenseMatrix)dhParam.Last().JointTransform.DenseMatrix.Inverse());

			double angleDeg1Plus = Math.Atan2(wristToAxis1.Matrix3D.OffsetY, wristToAxis1.Matrix3D.OffsetX) * 180 / Math.PI;
			double angleDeg1Minus = Math.Atan2(-wristToAxis1.Matrix3D.OffsetY, -wristToAxis1.Matrix3D.OffsetX) * 180 / Math.PI;
			ResultAxisSolutions.Add(new double[] { angleDeg1Plus, angleDeg1Minus });
		}


		private void ComputeAngle23Solutions(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
		{
			// this whole crowd looks suspicius except for these ... angles
			// TODO: needs more commenting and finalization.
			var rootToAxis2Trans = new TransformationMatrix(dhParam.First().JointTransform.DenseMatrix * dhParam[1].JointStandardTransform.DenseMatrix);

			// this is the transformation from joint 2 to the wrist center point
			TransformationMatrix wristToAxis2 = new TransformationMatrix( 
				(DenseMatrix)rootToAxis2Trans.DenseMatrix.Inverse() * 
				targetMatrix.DenseMatrix * 
				(DenseMatrix)dhParam.Last().JointTransform.DenseMatrix.Inverse());
			
			double distanceXYPlane = Math.Sqrt(Math.Pow(wristToAxis2.Matrix3D.OffsetX, 2) + Math.Pow(wristToAxis2.Matrix3D.OffsetY, 2));
			double distanceZDirection = wristToAxis2.Matrix3D.OffsetZ;

			double c = Math.Sqrt(Math.Pow(distanceXYPlane, 2) + Math.Pow(distanceZDirection, 2));
			double b = dhParam[2].A;
			double a = Math.Sqrt(Math.Pow(dhParam[3].A, 2) + Math.Pow(dhParam[3].D, 2));
			
			double angle1 = Math.Acos(Math.Pow(a,2) - Math.Pow(b, 2) - Math.Pow(c, 2)) / (-2 * c * b);
			double angle2 = Math.Acos(Math.Pow(c, 2) - Math.Pow(b, 2) - Math.Pow(a, 2)) / (-2 * b * a);


			continue here mate


			Debug.Print("\n" + wristToAxis2);

			Console.WriteLine("Do some computation here");

			throw new NotImplementedException();
		}
	}
}
