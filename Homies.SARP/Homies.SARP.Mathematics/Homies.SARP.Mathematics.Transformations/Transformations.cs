
using System;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Mathematics.Transformations
{
	public static class Transformations
	{
		/// <summary>
		/// Returns a rotation matrix rotated around the x axis
		/// </summary>
		/// <param name="thetaRad">rotation angle specified in radians</param>
		/// <returns></returns>
		public static DenseMatrix GetRotMatrixX(double thetaRad)
		{
			var rotX = DenseMatrix.OfArray(new double[,]
			{
				{ 1, 0, 0, 0},
				{ 0, Math.Cos(thetaRad), -Math.Sin(thetaRad), 0},
				{ 0, Math.Sin(thetaRad), Math.Cos(thetaRad), 0},
				{ 0, 0, 0, 1 }
			});
			rotX.CoerceZero(0.000001);
			return rotX;
		}

		/// <summary>
		/// Returns a rotation matrix rotated around the y axis
		/// </summary>
		/// <param name="thetaRad">rotation angle specified in radians</param>
		/// <returns></returns>
		public static DenseMatrix GetRotMatrixY(double thetaRad)
		{
			var rotY = DenseMatrix.OfArray(new double[,]
			{
				{ Math.Cos(thetaRad), 0, Math.Sin(thetaRad), 0},
				{ 0, 1, 0, 0},
				{ -Math.Sin(thetaRad), 0, Math.Cos(thetaRad), 0},
				{ 0, 0, 0, 1 }
			});
			rotY.CoerceZero(0.000001);
			return rotY;
		}

		/// <summary>
		/// Returns a rotation matrix rotated around the z axis
		/// </summary>
		/// <param name="thetaRad">rotation angle specified in radians</param>
		/// <returns></returns>
		public static DenseMatrix GetRotMatrixZ(double thetaRad)
		{
			var rotZ = DenseMatrix.OfArray(new double[,]
			{
				{ Math.Cos(thetaRad), -Math.Sin(thetaRad), 0, 0},
				{ Math.Sin(thetaRad), Math.Cos(thetaRad), 0, 0},
				{ 0, 0, 1, 0},
				{ 0, 0, 0, 1 }
			});
			rotZ.CoerceZero(0.000001);
			return rotZ;
		}

		/// <summary>
		/// Returns a translation matrix
		/// </summary>
		/// <param name="x">translates in x direction</param>
		/// <param name="y">translates in y direction</param>
		/// <param name="z">translates in z direction</param>
		/// <returns></returns>
		public static DenseMatrix GetTranslationMatrix(double x, double y, double z)
		{
			var trans = DenseMatrix.OfArray(new double[,]
			{
				{ 1, 0, 0, x},
				{ 0, 1, 0, y},
				{ 0, 0, 1, z},
				{ 0, 0, 0, 1 }
			});
			trans.CoerceZero(0.000001);
			return trans;
		}
	}
}