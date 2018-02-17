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

			List<double> angles = _inverse.GetInverseKinematics(currentRobotTarget, _dhParam);

			Assert.IsTrue(angles[0].DoubleEquals(testAngleDeg));
		}
	}
}