using System;
using System.Collections.Generic;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Diagnostics;
using Homies.SARP.Common.Extensions;

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

		public void GetAxisValues(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
		{
			List<double> returnAnglesForTest = new List<double>();

			ComputeAngle1Solutions(targetMatrix, dhParam);
			ComputeAngle23Solutions(targetMatrix, dhParam);
			ComputeAngle456Solutions(targetMatrix, dhParam);
			
			//TODO: Implement "ChooseValidConfiguration()"
			//ChooseValidConfiguration()
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
			
			double distanceXYPlane = Math.Sqrt(Math.Pow(wristToAxis2.Matrix3D.OffsetY, 2) + Math.Pow(wristToAxis2.Matrix3D.OffsetZ, 2));
			double distanceZDirection = wristToAxis2.Matrix3D.OffsetX;

			double a = Math.Sqrt(Math.Pow(dhParam[3].A, 2) + Math.Pow(dhParam[3].D, 2));
			double b = dhParam[2].A;
			double c = Math.Sqrt(Math.Pow(distanceXYPlane, 2) + Math.Pow(distanceZDirection, 2));

			double angle1 = Math.Acos((Math.Pow(a,2) - Math.Pow(b, 2) - Math.Pow(c, 2)) / (-2 * c * b));
			double angle2 = Math.Acos((Math.Pow(c, 2) - Math.Pow(b, 2) - Math.Pow(a, 2)) / (-2 * b * a));

			double angle2Deg = 90-angle2.RadToDeg();

			double flipFlapLookForward = Math.Atan2(distanceZDirection, 1 * distanceXYPlane);
			double flipFlapLookBackward = Math.Atan2(distanceZDirection, -1 * distanceXYPlane);

			double lookForwardSolution1 = Math.PI / 2 - (flipFlapLookForward + angle1);
			double lookForwardSolution2 = Math.PI / 2 - angle2;
			double lookForwardSolution3 = Math.PI / 2 - (flipFlapLookForward - angle1);
			double lookForwardSolution4 = angle2 - Math.PI * 3 / 2;

			double lookBackwardSolution1 = Math.PI / 2 - (flipFlapLookBackward + angle1);
			double lookBackwardSolution2 = Math.PI / 2 - angle2;
			double lookBackwardSolution3 = Math.PI / 2 - (flipFlapLookBackward - angle1);
			double lookBackwardSolution4 = angle2 - Math.PI * 3/2;

			ResultAxisSolutions.Add(new double[] {
				lookForwardSolution1.RadToDeg() - dhParam[1].AngleOffset,
				lookForwardSolution2.RadToDeg() - dhParam[2].AngleOffset,
				lookForwardSolution3.RadToDeg() - dhParam[1].AngleOffset,
				lookForwardSolution4.RadToDeg() - dhParam[2].AngleOffset,
				lookBackwardSolution1.RadToDeg() - dhParam[1].AngleOffset,
				lookBackwardSolution2.RadToDeg() - dhParam[2].AngleOffset,
				lookBackwardSolution3.RadToDeg() - dhParam[1].AngleOffset,
				lookBackwardSolution4.RadToDeg() - dhParam[2].AngleOffset,
			});
		}

		private void ComputeAngle456Solutions(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
		{
			//TODO: this is only temporary to check the correctness of the algorithm
			var target = targetMatrix.DenseMatrix;

			var tempRes = dhParam[0].JointTransform.DenseMatrix *
			dhParam[1].JointTransform.DenseMatrix *
			dhParam[2].JointTransform.DenseMatrix;

			var rotMat4To6 = tempRes.Inverse() * target;
			
			double R23 = rotMat4To6[1, 2];
			double R22 = rotMat4To6[1, 1];
			double R21 = rotMat4To6[1, 0];
			double R33 = rotMat4To6[2, 2];
			double R13 = rotMat4To6[0, 2];

			double[] theta4 = new double[2];
			double[] theta5 = new double[2];
			double[] theta6 = new double[2];

			if (!R33.Equals(1.0))
			{
				theta5[0] = Math.Acos(R23);
				theta5[1] = -Math.Acos(R23);
			}

			if (Math.Sin(theta5[0]).Equals(0.0))
			{
				theta4[0] = -Math.Atan2(R33, -R13);
			}
			else
			{
				theta4[0] = -Math.Atan2(R33 / Math.Sin(theta5[0]), -R13 / Math.Sin(theta5[0]));
			}

			if (Math.Sin(theta5[1]).Equals(0.0))
			{
				theta4[1] = -Math.Atan2(R33, -R13);
			}
			else
			{
				theta4[1] = -Math.Atan2(R33 / Math.Sin(theta5[1]), -R13 / Math.Sin(theta5[1]));
			}

			if (Math.Sin(theta5[0]).Equals(0.0))
			{
				theta6[1] = Math.Atan2(-R22, R21);
			}
			else
			{
				theta6[1] = Math.Atan2(-R22 / Math.Sin(theta5[0]), R21 / Math.Sin(theta5[0]));
			}

			if (Math.Sin(theta5[1]).Equals(0.0))
			{
				theta6[0] = Math.Atan2(-R22, R21);
			}
			else
			{
				theta6[0] = Math.Atan2(-R22 / Math.Sin(theta5[1]), R21 / Math.Sin(theta5[1]));
			}

			ResultAxisSolutions.Add(new double[] { theta4[0].RadToDeg(), theta4[1].RadToDeg() });
			ResultAxisSolutions.Add(new double[] { theta5[0].RadToDeg(), theta5[1].RadToDeg() });
			ResultAxisSolutions.Add(new double[] { theta6[0].RadToDeg(), theta6[1].RadToDeg() });
		}
	}
}
