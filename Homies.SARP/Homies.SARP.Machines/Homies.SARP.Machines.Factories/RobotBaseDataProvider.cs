using System;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Machines.Factories
{
	public static class RobotBaseDataProvider
	{
		public static TransformationMatrix GetJoint6ToFlangeTransformation(RobotManufacturer robotBrand)
		{
			DenseMatrix dense = null;

			switch (robotBrand)
			{
				case RobotManufacturer.Kuka:
					dense = Transformations.GetRotMatrixY(Math.PI);
					break;
				case RobotManufacturer.ABB:
					break;
				case RobotManufacturer.Fanuc:
					break;
				case RobotManufacturer.Kawasaki:
					break;
				case RobotManufacturer.UR:
					break;
				case RobotManufacturer.Mitsubishi:
					break;
				default:
					break;
			}

			return new TransformationMatrix(dense);
		}
	}
}