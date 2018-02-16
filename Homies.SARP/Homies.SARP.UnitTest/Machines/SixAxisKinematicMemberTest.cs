using Homies.SARP.Common.Extensions;
using Homies.SARP.Machines.Factories;
using Homies.SARP.Machines.MachineStructures;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.UnitTest.Machines
{
	[TestClass]
	public class SixAxisKinematicMemberTest
	{
		Robot _testRobot;

		[TestInitialize]
		public void Initialization()
		{
			_testRobot = new Robot("testRobi", DHParameterFactory.GetDhParameterForRobot(RobotModels.Kuka_KR270_R2700));
		}

		[TestMethod]
		public void TestRobotTargetWristFrameInAlpha5()
		{
			TransformationMatrix target = _testRobot.CurrentTarget;
			TransformationMatrix wrist = _testRobot.CurrentWristFrame;

			TransformationMatrix relToA5 = new TransformationMatrix((DenseMatrix)wrist.DenseMatrix.Inverse() * target.DenseMatrix);

			double[] translation = { relToA5.Matrix3D.OffsetX, relToA5.Matrix3D.OffsetY, relToA5.Matrix3D.OffsetZ};
			double length = translation.L2Norm();

			Assert.IsTrue(length.DoubleEquals(240));
		}

		[TestMethod]
		public void TestRobotTargetWristFrameWIthAllAxesRotated()
		{
			
			_testRobot.SetAnglesInDegree(new List<double>() { 33, -65, -30, 12.45, 34.123, 56.76 });
			
			TransformationMatrix target = _testRobot.CurrentTarget;
			TransformationMatrix wrist = _testRobot.CurrentWristFrame;

			TransformationMatrix relToA5 = new TransformationMatrix((DenseMatrix)wrist.DenseMatrix.Inverse() * target.DenseMatrix);

			double[] translation = { relToA5.Matrix3D.OffsetX, relToA5.Matrix3D.OffsetY, relToA5.Matrix3D.OffsetZ };
			double length = translation.L2Norm();

			Assert.IsTrue(length.DoubleEquals(240));
		}

		[TestMethod]
		public void TestRobotTargetWristFrameWIthAllAxesRotatedAtRandomAngles()
		{
			Random rand = new Random();

			var randAngles = new List<double>
			{
				rand.NextDouble() * 90,
				rand.NextDouble() * 90,
				rand.NextDouble() * 90,
				rand.NextDouble() * 90,
				rand.NextDouble() * 90,
				rand.NextDouble() * 90,
			};

			_testRobot.SetAnglesInDegree(randAngles);

			TransformationMatrix target = _testRobot.CurrentTarget;
			TransformationMatrix wrist = _testRobot.CurrentWristFrame;

			TransformationMatrix relToA5 = new TransformationMatrix((DenseMatrix)wrist.DenseMatrix.Inverse() * target.DenseMatrix);

			double[] translation = { relToA5.Matrix3D.OffsetX, relToA5.Matrix3D.OffsetY, relToA5.Matrix3D.OffsetZ };
			double length = translation.L2Norm();

			Assert.IsTrue(length.DoubleEquals(240));
		}
	}
}