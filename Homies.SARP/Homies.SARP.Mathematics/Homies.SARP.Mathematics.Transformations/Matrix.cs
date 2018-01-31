using System;
using System.Diagnostics;
using System.Windows.Media.Media3D;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Mathematics.Transformations
{
    [DebuggerDisplay("{ToString()}")]
    public class Matrix
    {
        #region FIELDS
        DenseMatrix _denseMatrix;
        Matrix3D _matrix3D;
        #endregion

        #region PROPERTIES
        public DenseMatrix DenseMatrix { get => _denseMatrix; set => _denseMatrix = value; }

        //TODO: Matrix3D private schalten, da wir mit GetMatrix3D öffentlichen Zugang ermöglichen?
        public Matrix3D Matrix3D { get => _matrix3D; set => _matrix3D = value; }
        #endregion

        public Matrix(DenseMatrix matrix = null)
        {
            DenseMatrix = matrix;
        }

        public Matrix3D GetMatrix3D()
        {
            throw new NotImplementedException();
        }

        public override string ToString()
        {
            if (DenseMatrix != null)
            {
                return DenseMatrix.ToString();
            }

            return ("DenseMatrix is null\t" + base.ToString());
        }

        public override bool Equals(object obj)
        {
            if (obj is Matrix mat)
            {
                return DenseMatrix.Equals(mat.DenseMatrix);
            }

            return false;
        }

        //TODO: Der override ist überflüssig, wenn nur base aufgerufen wird, oder?
        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}
