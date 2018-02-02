using System;
using System.Collections.Generic;
using Homies.SARP.Kinematics.Homies.SARP.Kinematics.Forward;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Machines.Factories;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Diagnostics;

namespace Homies.SARP.UnitTest.Kinematics
{
    [TestClass]
    public class KinematicsTests
    {
		List<DHParameter> _dhParam;

		[TestInitialize]
		public void InitializeKinematicsTestData()
		{
			_dhParam = DHParameterFactory.GetDhParameterForRobot(RobotModel.Kuka_KR270_R2700);
		}

        [TestMethod]
        public void CreateValidKinematic()
        {
			List<TransformationMatrix> matrices = new List<TransformationMatrix>();

			foreach (var dhParam in _dhParam)
			{
				DenseMatrix matrix = Transformations.GetTranslationMatrix(dhParam.A, 0, 0) * Transformations.GetRotMatrixX(dhParam.Alpha) * Transformations.GetRotMatrixZ(dhParam.Theta) * Transformations.GetTranslationMatrix(0, 0, dhParam.D);
				var transMatrix = new TransformationMatrix(matrix);
				matrices.Add(transMatrix);
			}

			DenseMatrix terminalFrameJointA6 = DenseMatrix.CreateIdentity(4);
			foreach (var jointMatrix in matrices)
			{
				terminalFrameJointA6 *= jointMatrix.DenseMatrix;
			}

			Debug.Print(String.Format("x = {0}\ny = {1}\nz = {2}\n", terminalFrameJointA6[0,3], terminalFrameJointA6[1, 3], terminalFrameJointA6[2, 3]));
			Debug.Print("\n\n" + terminalFrameJointA6.ToString());
			var x = 5;

        }


    }
}
