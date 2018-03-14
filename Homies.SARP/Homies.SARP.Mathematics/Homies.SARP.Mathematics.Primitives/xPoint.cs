using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace Homies.SARP.Mathematics.Primitives
{
	public class XPoint
	{
		#region FIELDS

		Point3D _mediaPoint3D = new Point3D(0, 0, 0);
		DenseVector _densePoint = DenseVector.OfArray(new double[] { 0, 0, 0, 1 });
		
		#endregion // FIELDS

		#region PROPERTIES
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
				if (value.Count < 3)
				{
					throw new ArgumentOutOfRangeException("Vector has wrong dimension");
				}

				_densePoint = DenseVector.OfArray(new double[] { value[0], value[1], value[2], 1 });
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

		#endregion //PROPERTIES

		#region INITIALIZATION

		public XPoint(double x, double y, double z)
		{
			SetValues(x, y, z);
		}

		public XPoint(DenseVector point)
		{
			if (point.Count < 3 || point.Count > 4)
			{
				throw new ArgumentOutOfRangeException("Dimension of the point is not plausible.");
			}

			SetValues(point[0], point[1], point[2]);
		}

		public XPoint(Point3D point)
		{
			SetValues(point.X, point.Y, point.Z);
		}

		private void SetValues(double x, double y, double z)
		{
			MediaPoint3D = new Point3D(x, y, z);
			DensePoint3D = DenseVector.OfArray(new double[] { x, y, z, 1});
			X = x; Y = y; Z = z;
		}

		#endregion //INITIALIZATION
	}
}