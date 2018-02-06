using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Homies.SARP.Kinematics.Homies.SARP.Kinematics.Forward;
using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Machines.Factories;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Homies.SARP.Machines.MachineStructures;
using Homies.SARP.Kinematics.Homies.SARP.Kinematics.Inverse;

namespace Homies.SARP.UnitTest.Kinematics
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
			DenseMatrix target = Transformations.GetTranslationMatrix(2000, 0, 2000) * Transformations.GetRotMatrixY(-Math.PI / 2);
			var targetFrame = new TransformationMatrix(target);

			List<double> solution = _inverse.GetInverseKinematics(targetFrame, _testRobot);

			if (solution.Count != _testRobot.Joints.Count)
			{
				throw new ArgumentOutOfRangeException();
			}

			for (int i = 0; i < solution.Count; i++)
			{
				_testRobot.Joints[i].DhParameter.Theta = solution[i];
			}

			TransformationMatrix terminalFrame = _testRobot.TCP;
			var res = target * terminalFrame.DenseMatrix.Inverse();

			//checking for dummy angles - correct angles must be set
			Assert.IsTrue(Math.Abs(res.Determinant() - 1) < 0.000001);
		}
	}
}