﻿using System;
using System.Collections.Generic;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Diagnostics;
using Homies.SARP.Common.Extensions;
using Homies.SARP.Mathematics.Primitives;

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


		//TODO: computation of angle 2 + 3 is dependent ont the wrist point relative to axis 2
		// when lloking backward wristpoint is completly different. need to fix that up.
		public void GetAngles1To3(XPoint wristPoint, List<DHParameter> dhParam)
		{
			var wristToAxis1 = new XPoint(dhParam.First().JointStandardTransform.DenseMatrix * wristPoint.DensePoint3D);

			double angleDeg1Plus = Math.Atan2(wristToAxis1.Y, wristToAxis1.X) * 180 / Math.PI;
			double angleDeg1Minus = Math.Atan2(-wristToAxis1.Y, -wristToAxis1.X) * 180 / Math.PI;
			ResultAxisSolutions.Add(new double[] { angleDeg1Plus, angleDeg1Minus });

			// TODO: needs more commenting and finalization.
			var rootToAxis2Trans = new TransformationMatrix(dhParam.First().JointTransform.DenseMatrix * dhParam[1].JointStandardTransform.DenseMatrix);

			// this is the transformation from joint 2 to the wrist center point
			XPoint wristToAxis2 = new XPoint(
				(DenseMatrix)rootToAxis2Trans.DenseMatrix.Inverse() *
				wristPoint.DensePoint3D);

			double distanceXYPlane = Math.Sqrt(Math.Pow(wristToAxis2.Y, 2) + Math.Pow(wristToAxis2.Z, 2));
			double distanceZDirection = wristToAxis2.X;

			double a = Math.Sqrt(Math.Pow(dhParam[3].A, 2) + Math.Pow(dhParam[3].D, 2));
			double b = dhParam[2].A;
			double c = Math.Sqrt(Math.Pow(distanceXYPlane, 2) + Math.Pow(distanceZDirection, 2));

			double angle1 = Math.Acos((Math.Pow(a, 2) - Math.Pow(b, 2) - Math.Pow(c, 2)) / (-2 * c * b));
			double angle2 = Math.Acos((Math.Pow(c, 2) - Math.Pow(b, 2) - Math.Pow(a, 2)) / (-2 * b * a));

			double angle2Deg = 90 - angle2.RadToDeg();

			double flipFlapLookForward = Math.Atan2(distanceZDirection, 1 * distanceXYPlane);
			double flipFlapLookBackward = Math.Atan2(distanceZDirection, -1 * distanceXYPlane);

			double lookForwardSolution1 = Math.PI / 2 - (flipFlapLookForward + angle1);
			double lookForwardSolution2 = Math.PI / 2 - angle2;
			double lookForwardSolution3 = Math.PI / 2 - (flipFlapLookForward - angle1);
			double lookForwardSolution4 = angle2 - Math.PI*1.5;

			double lookBackwardSolution1 = Math.PI / 2 - (flipFlapLookBackward + angle1);
			double lookBackwardSolution2 = Math.PI / 2 - angle2;
			double lookBackwardSolution3 = Math.PI / 2 - (flipFlapLookBackward - angle1);
			double lookBackwardSolution4 = angle2 - Math.PI * 1.5;

			ResultAxisSolutions.Add(new double[] {
				lookForwardSolution1.RadToDeg() - dhParam[1].AngleOffset,  //look forward elbow up axis 2
				lookForwardSolution3.RadToDeg() - dhParam[1].AngleOffset,  //look forward elbow down axis 2
				lookBackwardSolution1.RadToDeg() - dhParam[1].AngleOffset, //look backward elbow up axis 2
				lookBackwardSolution3.RadToDeg() - dhParam[1].AngleOffset, //look backward elbow down axis 2
			});

			ResultAxisSolutions.Add(new double[] {
				lookForwardSolution2.RadToDeg() - dhParam[2].AngleOffset,  //look forward elbow up axis 3
				lookForwardSolution4.RadToDeg() - dhParam[2].AngleOffset,  //look forward elbow down axis 3
				lookBackwardSolution2.RadToDeg() - dhParam[2].AngleOffset, //look backward elbow up axis 3
				lookBackwardSolution4.RadToDeg() - dhParam[2].AngleOffset, //look backward elbow down axis 3
			});
		}

		public void GetAngles4To6(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
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

			if (Math.Sin(theta5[0]).DoubleEquals(0.0))
			{
				theta4[0] = -Math.Atan2(R33, -R13);
			}
			else
			{
				theta4[0] = -Math.Atan2(R33 / Math.Sin(theta5[0]), -R13 / Math.Sin(theta5[0]));
			}

			if (Math.Sin(theta5[1]).DoubleEquals(0.0))
			{
				theta4[1] = -Math.Atan2(R33, -R13);
			}
			else
			{
				theta4[1] = -Math.Atan2(R33 / Math.Sin(theta5[1]), -R13 / Math.Sin(theta5[1]));
			}

			if (Math.Sin(theta5[0]).DoubleEquals(0.0))
			{
				theta6[1] = Math.Atan2(R22, R21);
			}
			else
			{
				theta6[1] = Math.Atan2(R22 / Math.Sin(theta5[0]), R21 / Math.Sin(theta5[0]));
			}

			if (Math.Sin(theta5[1]).DoubleEquals(0.0))
			{
				theta6[0] = Math.Atan2(R22, R21);
			}
			else
			{
				theta6[0] = Math.Atan2(R22 / Math.Sin(theta5[1]), R21 / Math.Sin(theta5[1]));
			}

			ResultAxisSolutions.Add(new double[] { theta4[0].RadToDeg(), theta4[1].RadToDeg() });
			ResultAxisSolutions.Add(new double[] { theta5[0].RadToDeg(), theta5[1].RadToDeg() });
			ResultAxisSolutions.Add(new double[] { theta6[0].RadToDeg(), theta6[1].RadToDeg() });
		}
	}
}
