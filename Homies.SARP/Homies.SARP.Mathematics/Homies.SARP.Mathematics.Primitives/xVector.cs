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
		Vector3D _mediaVector3D = new Vector3D(0, 0, 0);
		DenseVector _denseVector = DenseVector.OfArray(new double[] { 0, 0, 0});

		public Vector3D MediaVector3D
		{ get { return _mediaVector3D; }
			set
			{
				_mediaVector3D = value;
				_denseVector = DenseVector.OfArray(new double[] { value.X, value.Y, value.Z });
			}
		}
		public DenseVector DenseVector3D {
			get { return _denseVector; }
			set
			{
				if (value.Count != 3)
				{
					throw new ArgumentOutOfRangeException("Vector has wrong dimension");
				}

				_denseVector = value;
				_mediaVector3D = new Vector3D(_denseVector[0], _denseVector[1], _denseVector[2]);
			}
		}

		public double X {
			get { return _mediaVector3D.X; }
			set
			{
				_mediaVector3D.X = value;
				_denseVector[0] = value;
			}
		}

		public double Y
		{
			get { return _mediaVector3D.Y; }
			set
			{
				_mediaVector3D.Y = value;
				_denseVector[1] = value;
			}
		}

		public double Z
		{
			get { return _mediaVector3D.Z; }
			set
			{
				_mediaVector3D.Z = value;
				_denseVector[2] = value;
			}
		}

		public double L2Norm { get { return MediaVector3D.Length; } }

		public XVector(double x, double y, double z) : this(new Vector3D(x,y,z)) { }

		public XVector(Vector3D vector)
		{
			MediaVector3D = vector;
			DenseVector3D = DenseVector.OfArray(new double[] 
			{
				MediaVector3D.X,
				MediaVector3D.Y,
				MediaVector3D.Z
			});

			X = MediaVector3D.X;
			Y = MediaVector3D.Y;
			Z = MediaVector3D.Z;
		}
	}
}