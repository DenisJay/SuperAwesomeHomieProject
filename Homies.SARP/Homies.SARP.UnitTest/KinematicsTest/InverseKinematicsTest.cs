using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Homies.SARP.Kinematics.Forward;
using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Machines.Factories;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Homies.SARP.Machines.MachineStructures;
using Homies.SARP.Kinematics.Inverse;
using Homies.SARP.Common.Extensions;
using Homies.SARP.Kinematics.Common;

namespace Homies.SARP.UnitTest.KinematicsTest
{
	[TestClass]
	public class InverseKinematicsTest
	{
		Robot _testRobot;
		InverseKinematics _inverse;
		List<DHParameter> _dhParam;
		double[] testAnglesDeg;
		double[] testAnglesRad;

		[TestInitialize]
		public void InitializeStructure()
		{
			_dhParam = DHParameterFactory.GetDhParameterForRobot(RobotModels.Kuka_KR270_R2700);
			_testRobot = new Robot("testRobi", RobotManufacturer.Kuka, RobotModels.Kuka_KR270_R2700);
			_testRobot.SetJoint6ToFlangeTcpTrafo(RobotManufacturer.Kuka);
			_inverse = new InverseKinematics();

			var rand = new Random(DateTime.Now.Millisecond);

			testAnglesDeg = new double[] { 30 * rand.NextDouble(), -80 - 30*rand.NextDouble(), 30 * rand.NextDouble(), 30 * rand.NextDouble(), 30 * rand.NextDouble(), 30 * rand.NextDouble() };
			testAnglesRad = new double[] {
				testAnglesDeg[0].DegToRad(),
				testAnglesDeg[1].DegToRad(),
				testAnglesDeg[2].DegToRad(),
				testAnglesDeg[3].DegToRad(),
				testAnglesDeg[4].DegToRad(),
				testAnglesDeg[5].DegToRad()
			};

			for (int i = 0; i < testAnglesRad.Length; i++)
			{
				_testRobot.Joints[i].DhParameter.Theta = testAnglesRad[i];
			}

			_testRobot.SetJoint6ToFlangeTcpTrafo(RobotManufacturer.Kuka);
		}
		
		[TestMethod]
		public void TestInverseKinematicAngle1()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);
			Assert.IsTrue(testAnglesDeg[0].DoubleEquals(_testRobot.InvKin.ResultAxisSolutions[0][0]));
		}

		[TestMethod]
		public void TestInverseKinematicAngle2()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);
			Assert.IsTrue(testAnglesDeg[1].DoubleEquals(_testRobot.InvKin.ResultAxisSolutions[1][0]));
		}

		[TestMethod]
		public void TestInverseKinematicAngle3()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);
			Assert.IsTrue(testAnglesDeg[2].DoubleEquals(_testRobot.InvKin.ResultAxisSolutions[2][0]));
		}

		[TestMethod]
		public void TestInverseKinematicAngle4()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);
			Assert.IsTrue(testAnglesDeg[3].DoubleEquals(_testRobot.InvKin.ResultAxisSolutions[3][0]));
		}

		[TestMethod]
		public void TestInverseKinematicAngle5()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);
			Assert.IsTrue(testAnglesDeg[4].DoubleEquals(_testRobot.InvKin.ResultAxisSolutions[4][0]));
		}

		[TestMethod]
		public void TestInverseKinematicAngle6()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);
			Assert.IsTrue(testAnglesDeg[5].DoubleEquals(_testRobot.InvKin.ResultAxisSolutions[5][0]));
		}

		[TestMethod]
		public void TestForwardBackwardTarget()
		{
			_testRobot.SetAnglesInDegree(new List<double>() { 5, -110, 0, 0, 0, 0 }); 

			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);
			var originalFrame = new TransformationMatrix(_testRobot.CurrentWrist.DenseMatrix);
			
			Debug.Print(_testRobot.CurrentWrist.DenseMatrix.ToString());

			var axisAngles = new List<double>() {
				_testRobot.InvKin.ResultAxisSolutions[0][1],
				_testRobot.InvKin.ResultAxisSolutions[1][2],
				_testRobot.InvKin.ResultAxisSolutions[2][1],
				_testRobot.InvKin.ResultAxisSolutions[3][0],
				_testRobot.InvKin.ResultAxisSolutions[4][0],
				_testRobot.InvKin.ResultAxisSolutions[5][0]
			};

			_testRobot.SetAnglesInDegree(axisAngles);

			Debug.Print(_testRobot.CurrentWrist.DenseMatrix.ToString());
			Debug.Print(_testRobot.Joints[0].JointValue.RadToDeg().ToString());

			var newFrame = _testRobot.CurrentWrist;
			var res = newFrame.DenseMatrix - originalFrame.DenseMatrix;
			double sumResult = res.ToColumnMajorArray().Sum();

			Debug.Assert(sumResult.DoubleEquals(0.0));
		}

		[TestMethod]
		public void TestElbowUpElbowDownTarget()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);

			// check whether the elbow down configuration 
			// has the same same target as the elbow up configuration
			Debug.Assert(false);
		}

		[TestMethod]
		public void TestWristFlippedTarget()
		{
			_testRobot.ComputeAnglesForTargetFrame(_testRobot.CurrentTarget);

			// check whether the wrist flipped configuration 
			// has the same target as the wrist unflipped configuration
			Debug.Assert(false);
		}

	}
}