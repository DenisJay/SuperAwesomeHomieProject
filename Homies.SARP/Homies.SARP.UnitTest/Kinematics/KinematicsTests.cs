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
using Homies.SARP.Common.Extensions;

namespace Homies.SARP.UnitTest.Kinematics
{
	[TestClass]
	public class KinematicsTests
	{

		List<DHParameter> _dhParam;

		[TestInitialize]
		public void InitializeTestVariables()
		{
			_dhParam = DHParameterFactory.GetDhParameterForRobot(RobotModels.Kuka_KR270_R2700);
		}

		#region CheckDHParameter

		[TestMethod]
		public void CheckDHParameterValuesAlpha5Config()
		{
			List<TransformationMatrix> matrices = new List<TransformationMatrix>();
			DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
			var terminalMatrix = new TransformationMatrix();

			resMatrix = GetTerminalFrameFor(new List<double>() { 0, -Math.PI / 2, 0, 0, 0, 0 });
			
			terminalMatrix.DenseMatrix = resMatrix;
			Debug.Print("\n" + resMatrix.ToString());

			Assert.IsTrue(
				terminalMatrix.Matrix3D.OffsetX.DoubleEquals(1790) &&
				terminalMatrix.Matrix3D.OffsetY.DoubleEquals(0) &&
				terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784));
		}

		[TestMethod]
		public void CheckDHParameterValuesWristDownConfig()
		{
			List<TransformationMatrix> matrices = new List<TransformationMatrix>();
			DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
			var terminalMatrix = new TransformationMatrix();

			resMatrix = GetTerminalFrameFor(new List<double>() { 0, -Math.PI / 2, 0, 0, Math.PI / 2, Math.PI / 2 });

			terminalMatrix.DenseMatrix = resMatrix;
			Debug.Print("\n" + resMatrix.ToString());

			Assert.IsTrue(terminalMatrix.Matrix3D.OffsetX.DoubleEquals(1790-240) &&
				terminalMatrix.Matrix3D.OffsetY.DoubleEquals(0) &&
				terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784-240));
		}

		[TestMethod]
		public void CheckDHParameterValuesA1_90()
		{
			List<TransformationMatrix> matrices = new List<TransformationMatrix>();
			DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
			var terminalMatrix = new TransformationMatrix();

			resMatrix = GetTerminalFrameFor(new List<double>() { Math.PI / 2, -Math.PI / 2, 0, 0, 0, 0 });

			terminalMatrix.DenseMatrix = resMatrix;
			Debug.Print("\n" + resMatrix.ToString());

			Assert.IsTrue(terminalMatrix.Matrix3D.OffsetX.DoubleEquals(0) &&
				terminalMatrix.Matrix3D.OffsetY.DoubleEquals(-1790) &&
				terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784));
		}

		[TestMethod]
		public void CheckDHParameterValuesA1_45()
		{
			List<TransformationMatrix> matrices = new List<TransformationMatrix>();
			DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
			var terminalMatrix = new TransformationMatrix();

			resMatrix = GetTerminalFrameFor(new List<double>() { Math.PI / 4, -Math.PI / 2, 0, 0, 0, 0 });

			terminalMatrix.DenseMatrix = resMatrix;
			Debug.Print("\n" + resMatrix.ToString());

			Assert.IsTrue(
				terminalMatrix.Matrix3D.OffsetX.DoubleEquals(Math.Sin(Math.PI / 4) * 1790) &&
				terminalMatrix.Matrix3D.OffsetY.DoubleEquals(Math.Sin(Math.PI / 4) * -1790) &&
				terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784));
		}

		[TestMethod]
		public void CheckDHParameterValuesStretch()
		{
			List<TransformationMatrix> matrices = new List<TransformationMatrix>();
			DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
			var terminalMatrix = new TransformationMatrix();

			resMatrix = GetTerminalFrameFor(new List<double>() { 0, 0, -Math.PI / 2, 0, 0, 0 });

			terminalMatrix.DenseMatrix = resMatrix;
			Debug.Print("\n" + resMatrix.ToString());

			Assert.IsTrue(terminalMatrix.Matrix3D.OffsetX.DoubleEquals(2940) &&
				terminalMatrix.Matrix3D.OffsetY.DoubleEquals(0) &&
				terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(634));
		}

		[TestMethod]
		public void CheckDHParameterValuesWrist()
		{
			List<TransformationMatrix> matrices = new List<TransformationMatrix>();
			DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
			var terminalMatrix = new TransformationMatrix();

			resMatrix = GetTerminalFrameFor(new List<double>() { 0, -Math.PI / 2, 0 , Math.PI / 2, Math.PI / 2, 0 });

			terminalMatrix.DenseMatrix = resMatrix;
			Debug.Print("\n" + resMatrix.ToString());

			Assert.IsTrue(terminalMatrix.Matrix3D.OffsetX.DoubleEquals(1550) &&
				terminalMatrix.Matrix3D.OffsetY.DoubleEquals(-240) &&
				terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784));
		}

		#endregion

		#region Private Methods

		private DenseMatrix GetTerminalFrameFor(List<double> angles)
		{
			var resMatrix = DenseMatrix.CreateIdentity(4);

			if (angles.Count != _dhParam.Count)
			{
				return resMatrix;
			}

			for (int i = 0; i < _dhParam.Count; i++)
			{
				_dhParam[i].Theta = angles[i];

				DenseMatrix mat = Transformations.GetRotMatrixX(_dhParam[i].Alpha) *
					Transformations.GetTranslationMatrix(_dhParam[i].A, 0, 0) *
					Transformations.GetRotMatrixZ(_dhParam[i].Theta) *
					Transformations.GetTranslationMatrix(0, 0, _dhParam[i].D);
				resMatrix *= mat;
			}

			return resMatrix;
		}

		#endregion
	}
}
