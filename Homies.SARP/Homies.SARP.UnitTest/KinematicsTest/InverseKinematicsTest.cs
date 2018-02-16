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
using Homies.SARP.Kinematics.Homies.SARP.Kinematics.Inverse;
using Homies.SARP.Common.Extensions;

namespace Homies.SARP.UnitTest.KinematicsTest
{
	[TestClass]
	public class InverseKinematicsTest
	{
		Robot _testRobot;
		InverseKinematics _inverse;

		[TestInitialize]
		public void InitializeStructure()
		{
			var dhParams = DHParameterFactory.GetDhParameterForRobot(RobotModels.Kuka_KR270_R2700);
			_testRobot = new Robot("testRobi", dhParams);
			_inverse = new InverseKinematics();
		}

		[TestMethod]
		public void GetAnglesFromTargetFrame()
		{
			double testAngleValue = 0;
            _testRobot.Joints[0].DhParameter.Theta = testAngleValue;
			TransformationMatrix currentRobotTarget = _testRobot.CurrentTarget;

			List<double> angles = _inverse.GetInverseKinematics(currentRobotTarget);

			Assert.IsTrue(angles[0].DoubleEquals(testAngleValue));
		}
	}
}