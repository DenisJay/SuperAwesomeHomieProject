﻿using System;
using System.Windows.Media.Media3D;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Homies.SARP.Common.Extensions;
using Homies.SARP.Mathematics.Transformations;

namespace Homies.SARP.UnitTest.Mathematics
{
	[TestClass]
	public class MatrixTests
	{
		#region FIELDS
		TransformationMatrix _testRotationMatrix;
		TransformationMatrix _testTranslationMatrix;
		Vector3D _testVector3D;
		Point3D _testPoint3D;
		Vector _testVector;
		Vector _testPoint;
		#endregion

		#region CONSTRUCT

		[TestInitialize]
		public void InitializeTestData()
		{
			_testRotationMatrix = new TransformationMatrix(Transformations.GetRotMatrixZ(Math.PI));
			_testTranslationMatrix = new TransformationMatrix(Transformations.GetTranslationMatrix(100,50,25));

			_testPoint3D = new Point3D(7, 9, 12);
			_testVector3D = new Vector3D(2, 4, 6);

			_testPoint = DenseVector.OfArray(new double[] { _testPoint3D.X, _testPoint3D.Y, _testPoint3D.Z, 1 });
			_testVector = DenseVector.OfArray(new double[] { _testVector3D.X, _testVector3D.Y, _testVector3D.Z, 0 });
		}

		#endregion

		[TestMethod]
		public void CreateNewValidMatrix()
		{
			//Arrange
			var mat = new TransformationMatrix();

			//Assert
			Assert.IsFalse(mat == null);
			Assert.IsTrue(mat.Matrix3D == Matrix3D.Identity);
			Assert.IsFalse(mat.DenseMatrix == null);
		}

		[TestMethod]
		public void DenseMatrixAndMatrix3DHaveSameRotation()
		{
			Point3D pointResult3D = _testRotationMatrix.Matrix3D.Transform(_testPoint3D);
			Vector<double> pointResultDense = _testRotationMatrix.DenseMatrix * _testPoint;

			Assert.IsTrue(
                pointResult3D.X.DoubleEquals(pointResultDense[0]) &&
				pointResult3D.Y.DoubleEquals(pointResultDense[1]) && 
				pointResult3D.Z.DoubleEquals(pointResultDense[2]));
		}

		[TestMethod]
		public void DenseMatrixAndMatrix3DHaveSameTranslation()
		{
			Point3D pointResult3D = _testTranslationMatrix.Matrix3D.Transform(_testPoint3D);
			Vector<double> pointResultDense = _testTranslationMatrix.DenseMatrix * _testPoint;

			Assert.IsTrue(
				pointResult3D.X.DoubleEquals(pointResultDense[0]) &&
				pointResult3D.Y.DoubleEquals(pointResultDense[1]) &&
				pointResult3D.Z.DoubleEquals(pointResultDense[2]));
		}

		[TestMethod]
		public void DenseMatrixAndMatrix3DHaveSameRotationFromMatrix3D()
		{
			var rotation = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 180));
			_testRotationMatrix.Matrix3D = rotation.Value;

			Point3D pointResult3D = _testRotationMatrix.Matrix3D.Transform(_testPoint3D);
			Vector<double> pointResultDense = _testRotationMatrix.DenseMatrix * _testPoint;

			Assert.IsTrue(
				pointResult3D.X.DoubleEquals(pointResultDense[0]) &&
				pointResult3D.Y.DoubleEquals(pointResultDense[1]) &&
				pointResult3D.Z.DoubleEquals(pointResultDense[2]));
		}

		[TestMethod]
		public void DenseMatrixAndMatrix3DHaveSameTranslationFromMatrix3D()
		{
			var translation = new TranslateTransform3D(20, 20, 20);
			_testTranslationMatrix.Matrix3D = translation.Value;

			Point3D pointResult3D = _testTranslationMatrix.Matrix3D.Transform(_testPoint3D);
			Vector<double> pointResultDense = _testTranslationMatrix.DenseMatrix * _testPoint;

			Assert.IsTrue(
				pointResult3D.X.DoubleEquals(pointResultDense[0]) &&
				pointResult3D.Y.DoubleEquals(pointResultDense[1]) &&
				pointResult3D.Z.DoubleEquals(pointResultDense[2]));
		}


		[TestMethod]
		public void TestIdentity()
		{
			DenseMatrix mat1 = Transformations.GetRotMatrixX(Math.PI) * Transformations.GetRotMatrixZ(Math.PI / 2) * Transformations.GetTranslationMatrix(123, 13, 4235);
			var res = mat1 * mat1.Inverse();

			double det = res.Determinant();

			Assert.IsTrue(Math.Abs(res.Determinant() - 1) < 0.000001);
		}
	}
}
