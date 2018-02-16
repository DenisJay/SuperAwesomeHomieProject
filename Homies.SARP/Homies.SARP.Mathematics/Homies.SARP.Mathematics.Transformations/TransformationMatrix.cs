using System;
using System.Diagnostics;
using System.Windows.Media.Media3D;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Mathematics.Transformations
{
    [DebuggerDisplay("{ToString()}")]
    public class TransformationMatrix
    {
        #region FIELDS
        private DenseMatrix _denseMatrix;
        private Matrix3D _matrix3D;
        #endregion

        #region PROPERTIES
        public DenseMatrix DenseMatrix
        {
            get { return _denseMatrix; }
            set
            {
                _denseMatrix = value;
				_denseMatrix.CoerceZero(0.00000001);
                UpdateMatrix3D();
            }
        }

        public Matrix3D Matrix3D
        {
            get
            {
                return _matrix3D;
            }
            set
            {
                _matrix3D = value;
                UpdateDenseMatrix();
            }
        }

        #endregion

        public TransformationMatrix(DenseMatrix matrix = null)
        {
            if (matrix == null)
            {
                DenseMatrix = DenseMatrix.CreateIdentity(4);
            }
            else
            {
                DenseMatrix = matrix;
            }
        }


        /// <summary>
        /// 3D Matrices in .net framework are transposed compared to normal homogeneous transformation matrices
        /// </summary>
        private void UpdateMatrix3D()
        {
            var transposedDense = DenseMatrix.Transpose();

            Matrix3D = new Matrix3D()
            {
                M11 = transposedDense[0, 0],
                M12 = transposedDense[0, 1],
                M13 = transposedDense[0, 2],
                OffsetX = transposedDense[3, 0],
                M21 = transposedDense[1, 0],
                M22 = transposedDense[1, 1],
                M23 = transposedDense[1, 2],
                OffsetY = transposedDense[3, 1],
                M31 = transposedDense[2, 0],
                M32 = transposedDense[2, 1],
                M33 = transposedDense[2, 2],
                OffsetZ = transposedDense[3, 2],
            };
        }

        /// <summary>
        /// See commecnt at UpdateMatrix3D()
        /// </summary>
        private void UpdateDenseMatrix()
        {
            DenseMatrix[0, 0] = Matrix3D.M11;
            DenseMatrix[0, 1] = Matrix3D.M21;
            DenseMatrix[0, 2] = Matrix3D.M31;
            DenseMatrix[0, 3] = Matrix3D.OffsetX;
            DenseMatrix[1, 0] = Matrix3D.M12;
            DenseMatrix[1, 1] = Matrix3D.M22;
            DenseMatrix[1, 2] = Matrix3D.M32;
            DenseMatrix[1, 3] = Matrix3D.OffsetY;
            DenseMatrix[2, 0] = Matrix3D.M13;
            DenseMatrix[2, 1] = Matrix3D.M23;
            DenseMatrix[2, 2] = Matrix3D.M33;
            DenseMatrix[2, 3] = Matrix3D.OffsetZ;
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
            var mat = obj as TransformationMatrix;
            if (mat != null)
            {
                return DenseMatrix.Equals(mat.DenseMatrix);
            }

            return false;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }
}