using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using Homies.SARP.Common.Homies.SARP.Common.Extensions;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Kinematics.Forward
{
    public abstract class Kinematics : IForwardKinematics
    {
        #region Construct

        protected Kinematics(IReadOnlyCollection<DHParameter> dhParameter)
        {
            //A kinematic with no joints is invalid.
            if (dhParameter == null || !dhParameter.Any())
            {
                throw new ArgumentNullException(nameof(dhParameter));
            }

			DhParameterCollection = new SortedList<int, DHParameter>();

            for (var i = 0; i < dhParameter.Count; i++)
            {
				DhParameterCollection.Add(i, dhParameter.ElementAt(i));
            }
        }

        #endregion

        #region Methods

        /// <summary>
        /// Returns the position hit from the current configuration.
        /// </summary>
        public Matrix3D GetForwardTransformationMatrix()
        {
            var zAxis = new Vector3D(0, 0, 1);
            var xAxis = new Vector3D(1, 0, 0);

            var transformGroup = new Transform3DGroup();

            for (int i = 0; i < DhParameterCollection.Count; i++)
            {
                var curretDHParameter = DhParameterCollection[i];

                zAxis = transformGroup.Value.ZAxis();
                zAxis.Normalize();
                
                //Translation in Z
                var zTranslate = new TranslateTransform3D(zAxis * curretDHParameter.D);
                //Rotation around the z-Axis
                var thetaRotate = new RotateTransform3D(new AxisAngleRotation3D(zAxis, curretDHParameter.Theta * 180 / Math.PI));

                transformGroup.Children.Insert(0, zTranslate);
                transformGroup.Children.Insert(1, thetaRotate);

                xAxis = transformGroup.Value.XAxis();
                xAxis.Normalize();

                //Translation in X
                var xTranslate = new TranslateTransform3D(xAxis * curretDHParameter.A);
                //Rotation around X
                var alphaRotate = new RotateTransform3D(new AxisAngleRotation3D(xAxis, curretDHParameter.Alpha * 180 / Math.PI));

                //Alpha Rotation has to be first, as the x-Axis 
                transformGroup.Children.Insert(0, alphaRotate);
                transformGroup.Children.Insert(1, xTranslate);

            }

            return transformGroup.Value;
        }

		public abstract TransformationMatrix GetTerminalFrame(List<DHParameter> joints);
		public abstract TransformationMatrix GetTerminalFrame(List<DHParameter> joints, List<double> jointValues);
		public abstract int GetStatus(List<DHParameter> joints, List<double> jointValues);
		public abstract int GetTurn(List<DHParameter> joints, List<double> jointValues);

		/// <summary>
		/// Computes the terminal frame for a given kinematic chain using dh-parameter and any given axis configuration
		/// </summary>
		/// <param name="dhParams">Set of dh-parameter specifying the kinematic chain</param>
		/// <param name="angles">angles specifying the configuration</param>
		/// <returns></returns>
		public static DenseMatrix GetTerminalFrameFor(List<DHParameter> dhParams, List<double> angles)
		{
			var resMatrix = DenseMatrix.CreateIdentity(4);

			if (angles.Count != dhParams.Count)
			{
				throw new ArgumentOutOfRangeException("The number of joints and the given number of angles do not fit.");
			}

			for (int i = 0; i < dhParams.Count; i++)
			{
				dhParams[i].Theta = angles[i];

				DenseMatrix mat = Transformations.GetRotMatrixX(dhParams[i].Alpha) *
					Transformations.GetTranslationMatrix(dhParams[i].A, 0, 0) *
					Transformations.GetRotMatrixZ(dhParams[i].Theta) *
					Transformations.GetTranslationMatrix(0, 0, dhParams[i].D);
				resMatrix *= mat;
			}

			return resMatrix;
		}

		#endregion

		#region Properties

		public readonly SortedList<int, DHParameter> DhParameterCollection;

        #endregion

    }

}