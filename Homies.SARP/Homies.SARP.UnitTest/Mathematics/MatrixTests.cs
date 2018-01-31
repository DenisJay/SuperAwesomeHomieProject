using System;
using System.Windows.Media.Media3D;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.UnitTest.Extensions;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra;

namespace Homies.SARP.UnitTest.Mathematics
{
	[TestClass]
	public class MatrixTests
	{

		TransformationMatrix _testRotationMatrix;
		TransformationMatrix _testTranslationMatrix;
		Vector3D _testVector3D;
		Point3D _testPoint3D;
		Vector _testVector;
		Vector _testPoint;

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


		[TestMethod]
		public void CreateNewValidMatrix()
		{
			//Arrange
			var mat = new TransformationMatrix();

			//Assert
			Assert.IsFalse(mat is null);
			Assert.IsTrue(mat.Matrix3D == Matrix3D.Identity);
			Assert.IsFalse(mat.DenseMatrix is null);
		}

		[TestMethod]
		public void DenseMatrixAndMatrix3DHaveSameRotation()
		{
			Point3D pointResult3D = _testRotationMatrix.Matrix3D.Transform(_testPoint3D);
			Vector<double> pointResultDense = _testRotationMatrix.DenseMatrix * _testPoint;

			Assert.IsTrue(
				pointResult3D.X == pointResultDense[0] && 
				pointResult3D.Y == pointResultDense[1] && 
				pointResult3D.Z == pointResultDense[2]);
		}

		[TestMethod]
		public void DenseMatrixAndMatrix3DHaveSameTranslation()
		{
			Point3D pointResult3D = _testTranslationMatrix.Matrix3D.Transform(_testPoint3D);
			Vector<double> pointResultDense = _testTranslationMatrix.DenseMatrix * _testPoint;

			Assert.IsTrue(
				pointResult3D.X == pointResultDense[0] &&
				pointResult3D.Y == pointResultDense[1] &&
				pointResult3D.Z == pointResultDense[2]);
		}
	}
}
