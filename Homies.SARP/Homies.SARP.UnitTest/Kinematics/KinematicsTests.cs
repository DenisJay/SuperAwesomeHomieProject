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
using System.Windows.Media.Media3D;
using Homies.SARP.Common.Homies.SARP.Common.Extensions;

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

        /// <summary>
        /// Testing the forward kinematic for one joint.
        /// </summary>
        [TestMethod]
        public void TestForwardKinematic()
        {
            //Arrange
            var joints = new List<Joint>
            {
                new RotationalJoint(0, Math.PI, new DHParameter(Math.PI, 0, 0, 500))
            };

            //Act
            var kinematc = new RobotKinematics(joints);
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

            resMatrix = GetTerminalFrameFor(_dhParam, new List<double>() { 0, -Math.PI / 2, 0, 0, 0, 0 });

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

            resMatrix = GetTerminalFrameFor(_dhParam, new List<double>() { 0, -Math.PI / 2, 0, 0, Math.PI / 2, Math.PI / 2 });

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

            resMatrix = GetTerminalFrameFor(_dhParam, new List<double>() { Math.PI / 2, -Math.PI / 2, 0, 0, 0, 0 });

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

            resMatrix = GetTerminalFrameFor(_dhParam, new List<double>() { Math.PI / 4, -Math.PI / 2, 0, 0, 0, 0 });

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

            resMatrix = GetTerminalFrameFor(_dhParam, new List<double>() { 0, 0, -Math.PI / 2, 0, 0, 0 });

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

            resMatrix = GetTerminalFrameFor(_dhParam, new List<double>() { 0, -Math.PI / 2, 0, Math.PI / 2, Math.PI / 2, 0 });

            terminalMatrix.DenseMatrix = resMatrix;
            Debug.Print("\n" + resMatrix.ToString());

            Assert.IsTrue(terminalMatrix.Matrix3D.OffsetX.DoubleEquals(1550) &&
                terminalMatrix.Matrix3D.OffsetY.DoubleEquals(-240) &&
                terminalMatrix.Matrix3D.OffsetZ.DoubleEquals(1784));
        }



        #endregion

        #region Private Methods

		/// <summary>
		/// Computes the terminal frame for a given kinematic chain using dh-parameter and any given axis configuration
		/// </summary>
		/// <param name="dhParams">Set of dh-parameter specifying the kinematic chain</param>
		/// <param name="angles">angles specifying the configuration</param>
		/// <returns></returns>
        public static DenseMatrix GetTerminalFrameFor(List<DHParameter> dhParams, List<double> angles)
        {
            var resMatrix = DenseMatrix.CreateIdentity(4);

            if (angles.Count != dhParams.Count)
            {
				throw new ArgumentOutOfRangeException("The number of joints and the given number of angles do not fit.");
            }

            for (int i = 0; i < dhParams.Count; i++)
            {
				dhParams[i].Theta = angles[i];

                DenseMatrix mat = Transformations.GetRotMatrixX(dhParams[i].Alpha) *
                    Transformations.GetTranslationMatrix(dhParams[i].A, 0, 0) *
                    Transformations.GetRotMatrixZ(dhParams[i].Theta) *
                    Transformations.GetTranslationMatrix(0, 0, dhParams[i].D);
                resMatrix *= mat;
            }

            return resMatrix;
        }

        #endregion
    }
}
