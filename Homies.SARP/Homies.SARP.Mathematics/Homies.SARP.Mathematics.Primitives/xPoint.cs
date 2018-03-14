using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace Homies.SARP.Mathematics.Homies.SARP.Mathematics.Primitives
{
	public class XPoint
	{
		Point3D _mediaPoint3D = new Point3D(0, 0, 0);
		DenseVector _densePoint = DenseVector.OfArray(new double[] { 0, 0, 0, 1 });

		public Point3D MediaPoint3D
		{
			get { return _mediaPoint3D; }
			set
			{
				_mediaPoint3D = value;
				_densePoint = DenseVector.OfArray(new double[] { value.X, value.Y, value.Z, 1 });
			}
		}
		public DenseVector DensePoint3D
		{
			get { return _densePoint; }
			set
			{
				if (value.Count != 3)
				{
					throw new ArgumentOutOfRangeException("Vector has wrong dimension");
				}

				_densePoint = value;
				_mediaPoint3D = new Point3D(_densePoint[0], _densePoint[1], _densePoint[2]);
			}
		}

		public double X
		{
			get { return _mediaPoint3D.X; }
			set
			{
				_mediaPoint3D.X = value;
				_densePoint[0] = value;
			}
		}

		public double Y
		{
			get { return _mediaPoint3D.Y; }
			set
			{
				_mediaPoint3D.Y = value;
				_densePoint[1] = value;
			}
		}

		public double Z
		{
			get { return _mediaPoint3D.Z; }
			set
			{
				_mediaPoint3D.Z = value;
				_densePoint[2] = value;
			}
		}

		public double Length { get { return DensePoint3D.L2Norm(); } }

		public XPoint(double x, double y, double z) : this(new Point3D(x, y, z)) { }

		public XPoint(Point3D point)
		{
			MediaPoint3D = point;
			DensePoint3D = DenseVector.OfArray(new double[]
			{
				MediaPoint3D.X,
				MediaPoint3D.Y,
				MediaPoint3D.Z,
				1
			});

			X = MediaPoint3D.X;
			Y = MediaPoint3D.Y;
			Z = MediaPoint3D.Z;
		}
	}
}