using System;
using System.Collections.Generic;
using System.Linq;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Kinematics.Forward
{
    public abstract class ForwardKinematics : IForwardKinematics
    {
        #region Construct

        protected ForwardKinematics(IReadOnlyCollection<DHParameter> dhParameter)
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
        public DenseMatrix GetForwardTransformationMatrix()
        {

            var result = DenseMatrix.CreateIdentity(4);

            for (int i = 0; i < DhParameterCollection.Count; i++)
            {
                var currentDhParameter = DhParameterCollection[i];

                var currentMatrix = GetDenseMatrixForDhParameter(currentDhParameter);
                result *= currentMatrix;
            }

            return result;
        }

        /// <summary>
        /// Computes the transformation matrix for any given DHParameter.
        /// </summary>
        /// <param name="dhParam"> DHParameter for which the transformation matrix is to be determined. </param>
        /// <returns></returns>
        public static DenseMatrix GetDenseMatrixForDhParameter(DHParameter dhParam)
        {

            // Source: http://www.oemg.ac.at/Mathe-Brief/fba2015/VWA_Prutsch.pdf, Page 19
            // http://www4.cs.umanitoba.ca/~jacky/Robotics/Papers/spong_kinematics.pdf, Page 63
            // https://de.wikipedia.org/wiki/Denavit-Hartenberg-Transformation
            var firstColumn = DenseVector.OfArray(new double[] {
                Math.Cos(dhParam.Theta),
                Math.Sin(dhParam.Theta),
                0,
                0
            });

            var secondColumn = DenseVector.OfArray(new double[] {
                -Math.Sin(dhParam.Theta) * Math.Cos(dhParam.Alpha),
                Math.Cos(dhParam.Theta) * Math.Cos(dhParam.Alpha),
                Math.Sin(dhParam.Alpha),
                0
            });

            var thirdColumn = DenseVector.OfArray(new double[] {
                Math.Sin(dhParam.Theta) * Math.Sin(dhParam.Alpha),
                -Math.Cos(dhParam.Theta) * Math.Sin(dhParam.Alpha),
                Math.Cos(dhParam.Alpha),
                0
            });

            var fourthColumn = DenseVector.OfArray(new double[] {
                dhParam.A * Math.Cos(dhParam.Theta),
                dhParam.A * Math.Sin(dhParam.Theta),
                dhParam.D,
                1
            });

            var columns = new DenseVector[] { firstColumn, secondColumn, thirdColumn, fourthColumn };
            var matrix = DenseMatrix.OfColumnVectors(columns);

            return matrix;
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