using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MathNet.Numerics.LinearAlgebra.Double;
using Homies.SARP.Common.Extensions;
using Homies.SARP.Common.Homies.SARP.Common.Extensions;
using Homies.SARP.Kinematics.Common;
using Homies.SARP.Machines.Factories;
using Homies.SARP.Mathematics.Transformations;
using Kin = Homies.SARP.Kinematics.Forward;

namespace Homies.SARP.UnitTest.KinematicsTest
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

        /// <summary>
        /// Testing the forward kinematic for one joint.
        /// </summary>
        [TestMethod]
        public void TestForwardKinematic()
        {
            //Arrange
            var dhParams = new List<DHParameter>
            {
                new DHParameter(Math.PI, 0, 0, 500)
            };

            //Act
            var kinematc = new Kin.RobotKinematics(dhParams);
            var forwardMatrix = kinematc.GetForwardTransformationMatrix();

            //Assert
            //By rotating 180° around the x-Axis (Alpha = Pi), the z and y axis should be inverted.
            var xAxis = forwardMatrix.XAxis();
            var yAxis = forwardMatrix.YAxis();
            var zAxis = forwardMatrix.ZAxis();
            var offset = forwardMatrix.Offset();

            Assert.IsTrue(xAxis.X.DoubleEquals(1));
            Assert.IsTrue(xAxis.Y.DoubleEquals(0));
            Assert.IsTrue(xAxis.Z.DoubleEquals(0));

            Assert.IsTrue(yAxis.X.DoubleEquals(0));
            Assert.IsTrue(yAxis.Y.DoubleEquals(-1));
            Assert.IsTrue(yAxis.Z.DoubleEquals(0));

            Assert.IsTrue(zAxis.X.DoubleEquals(0));
            Assert.IsTrue(zAxis.Y.DoubleEquals(0));
            Assert.IsTrue(zAxis.Z.DoubleEquals(-1));

            Assert.IsTrue(offset.X.DoubleEquals(0));
            Assert.IsTrue(offset.Y.DoubleEquals(0));
            Assert.IsTrue(offset.Z.DoubleEquals(500));
        }

        #region CheckDHParameter

        [TestMethod]
        public void CheckDHParameterValuesAlpha5Config()
        {
            List<TransformationMatrix> matrices = new List<TransformationMatrix>();
            DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
            var terminalMatrix = new TransformationMatrix();

            resMatrix = Kin.Kinematics.GetTerminalFrameFor(_dhParam, new List<double>() { 0, -Math.PI / 2, 0, 0, 0, 0 });

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

            resMatrix = Kin.Kinematics.GetTerminalFrameFor(_dhParam, new List<double>() { 0, -Math.PI / 2, 0, 0, Math.PI / 2, Math.PI / 2 });

            terminalMatrix.DenseMatrix = resMatrix;
            Debug.Print("\n" + resMatrix.ToString());

            Assert.IsTrue(terminalMatrix.Matrix3D.OffsetX.DoubleEquals(1790 - 240) &&
                terminalMatrix.Matrix3D.OffsetY.DoubleEquals(0) &&
                terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784 - 240));
        }

        [TestMethod]
        public void CheckDHParameterValuesA1_90()
        {
            List<TransformationMatrix> matrices = new List<TransformationMatrix>();
            DenseMatrix resMatrix = DenseMatrix.CreateIdentity(4);
            var terminalMatrix = new TransformationMatrix();

            resMatrix = Kin.Kinematics.GetTerminalFrameFor(_dhParam, new List<double>() { Math.PI / 2, -Math.PI / 2, 0, 0, 0, 0 });

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

            resMatrix = Kin.Kinematics.GetTerminalFrameFor(_dhParam, new List<double>() { Math.PI / 4, -Math.PI / 2, 0, 0, 0, 0 });

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

            resMatrix = Kin.Kinematics.GetTerminalFrameFor(_dhParam, new List<double>() { 0, 0, -Math.PI / 2, 0, 0, 0 });

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

            resMatrix = Kin.Kinematics.GetTerminalFrameFor(_dhParam, new List<double>() { 0, -Math.PI / 2, 0, Math.PI / 2, Math.PI / 2, 0 });

            terminalMatrix.DenseMatrix = resMatrix;
            Debug.Print("\n" + resMatrix.ToString());

            Assert.IsTrue(terminalMatrix.Matrix3D.OffsetX.DoubleEquals(1550) &&
                terminalMatrix.Matrix3D.OffsetY.DoubleEquals(-240) &&
                terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784));
        }

        #endregion
    }
}
