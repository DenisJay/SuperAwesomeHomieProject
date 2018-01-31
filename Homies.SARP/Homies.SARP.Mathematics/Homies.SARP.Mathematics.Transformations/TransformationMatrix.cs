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
		DenseMatrix _denseMatrix;
		Matrix3D _matrix3D;
		#endregion

		#region PROPERTIES
		public DenseMatrix DenseMatrix
		{
			get { return _denseMatrix; }
			set
			{
				_denseMatrix = value;
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
			var tempDM = DenseMatrix.Transpose();

			Matrix3D = new Matrix3D()
			{
				M11 = tempDM[0, 0],
				M12 = tempDM[0, 1],
				M13 = tempDM[0, 2],
				OffsetX = tempDM[3, 0],
				M21 = tempDM[1, 0],
				M22 = tempDM[1, 1],
				M23 = tempDM[1, 2],
				OffsetY = tempDM[3, 1],
				M31 = tempDM[2, 0],
				M32 = tempDM[2, 1],
				M33 = tempDM[2, 2],
				OffsetZ = tempDM[3, 2],
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
			if (obj is TransformationMatrix mat)
			{
				return DenseMatrix.Equals(mat.DenseMatrix);
			}

			return false;
		}

		//TODO: Der override ist überflüssig, wenn nur base aufgerufen wird, oder?
		//der meckert bei mir, wenn man einen override fuer Equals hat, dass man auch einen für GetHashCode braucht
		public override int GetHashCode()
		{
			return base.GetHashCode();
		}
	}
}