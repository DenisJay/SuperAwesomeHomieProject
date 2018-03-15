﻿using System;
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

			testAnglesDeg = new double[] { 10, -80, 11, 12, 13, 14 };
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
	}
}