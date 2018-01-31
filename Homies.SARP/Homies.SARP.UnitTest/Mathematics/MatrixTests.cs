using System.Windows.Media.Media3D;
using Homies.SARP.UnitTest.Extensions;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Matrix = Homies.SARP.Mathematics.Transformations.Matrix;

namespace Homies.SARP.UnitTest.Mathematics
{
    [TestClass]
    public class MatrixTests
    {
        [TestMethod]
        public void CreateNewValidMatrix()
        {
            //Arrange
            var mat = new Matrix();

            //Act


            //Assert
            Assert.IsFalse(mat is null);
            Assert.IsTrue(mat.GetMatrix3D() == Matrix3D.Identity);
            Assert.IsFalse(mat.DenseMatrix is null);
        }
    }
}
