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

		[TestInitialize]
		public void InitializeStructure()
		{
			_dhParam = DHParameterFactory.GetDhParameterForRobot(RobotModels.Kuka_KR270_R2700);
			_testRobot = new Robot("testRobi", _dhParam);
			_inverse = new InverseKinematics();
		}

		[TestMethod]
		public void GetAnglesFromTargetFrame()
		{
			double testAngleDeg = 15;
			double testAngleRad = testAngleDeg * Math.PI / 180;
            _testRobot.Joints[0].DhParameter.Theta = testAngleRad;
			TransformationMatrix currentRobotTarget = _testRobot.CurrentTarget;

			_inverse.GetAxisValues(currentRobotTarget, _dhParam);

			Assert.IsTrue(_inverse.ResultAxisSolutions[0][0].DoubleEquals(testAngleDeg));
		}

		[TestMethod]
		public void GetAnglesAxis1To3FromTargetFrame()
		{
			double degToRad = Math.PI / 180;
			double[] testAnglesDeg = { 0, -90, 0, 12, 13, 14};
			double[] testAnglesRad = {
				0 * degToRad, -90 * degToRad, 0 * degToRad,
				12 * degToRad, 13 * degToRad, 14 * degToRad };

			for (int i = 0; i < testAnglesRad.Length; i++)
			{
				_testRobot.Joints[i].DhParameter.Theta = testAnglesRad[i];
			}

			TransformationMatrix currentRobotTarget = _testRobot.CurrentTarget;
			_inverse.GetAxisValues(currentRobotTarget, _dhParam);

			double[] angles = { _inverse.ResultAxisSolutions[0][0], _inverse.ResultAxisSolutions[1][0], _inverse.ResultAxisSolutions[1][1]};

			Assert.IsTrue(angles.DoublesEqual(testAnglesDeg));
		}
	}
}