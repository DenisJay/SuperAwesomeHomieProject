using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace Homies.SARP.Mathematics.Primitives
{
	public class XVector
	{

		#region FIELDS
		Vector3D _mediaVector3D = new Vector3D(0, 0, 0);
		DenseVector _denseVector = DenseVector.OfArray(new double[] { 0, 0, 0});
		#endregion //FIELDS

		#region PROPERTIES

		public Vector3D MediaVector3D
		{
			get { return _mediaVector3D; }
			set
			{
				SetValues(value.X, value.Y, value.Z );
			}
		}

		public DenseVector DenseVector3D {
			get { return _denseVector; }
			set
			{
				if (value.Count < 3)
				{
					throw new ArgumentOutOfRangeException("Vector has wrong dimension");
				}
				
				SetValues(_denseVector[0], _denseVector[1], _denseVector[2]);
			}
		}

		public double X {
			get { return _mediaVector3D.X; }
			set { SetValues(value, MediaVector3D.Y, MediaVector3D.Z); }
		}

		public double Y
		{
			get { return _mediaVector3D.Y; }
			set { SetValues(MediaVector3D.X, value, MediaVector3D.Z); }
		}

		public double Z
		{
			get { return _mediaVector3D.Z; }
			set { SetValues(MediaVector3D.X, MediaVector3D.Y, value); }
		}

		public double L2Norm { get { return MediaVector3D.Length; } }

		#endregion //PROPERTIES

		#region INITIALIZATION

		public XVector(double x, double y, double z)
		{
			SetValues(x, y, z);
		}

		public XVector(Vector3D vector)
		{
			SetValues(vector.X, vector.Y, vector.Z);
		}

		public XVector(DenseVector vector)
		{
			if (vector.Count < 3)
			{
				throw new ArgumentOutOfRangeException("Vector dimension is not correct.");
			}

			SetValues(vector[0], vector[1], vector[2]);
		}

		private void SetValues(double x, double y, double z)
		{
			X = x; Y = y; Z = z;
			MediaVector3D = new Vector3D(x, y, z);
			DenseVector3D = DenseVector.OfArray(new double[] { x, y, z, 0 });
		}

		#endregion //INITIALIZATION
	}
}