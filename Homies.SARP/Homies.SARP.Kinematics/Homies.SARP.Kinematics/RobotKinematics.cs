using System;
using System.Collections.Generic;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using Homies.SARP.Kinematics.Forward;
using Homies.SARP.Kinematics.Inverse;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Kinematics
{
    public class RobotKinematics : IForwardKinematics, IInverseKinematics
    {

        public RobotKinematics(IReadOnlyCollection<DHParameter> dhParameter)
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

        /// <summary>
        /// Returns the position hit from the current configuration.
        /// </summary>
        public DenseMatrix GetForwardTransformationMatrix()
        {
            
            var result = DenseMatrix.CreateIdentity(4);

            for (int i = 0; i < DhParameterCollection.Count; i++)
            {
                var currentDhParameter = DhParameterCollection[i];

                var currentMatrix = ForwardKinematics.GetDenseMatrixForDhParameter(currentDhParameter);
                result *= currentMatrix;
            }

            return result;

        }

        public TransformationMatrix GetTerminalFrame(List<DHParameter> joints)
        {
            throw new NotImplementedException();
        }

        public TransformationMatrix GetTerminalFrame(List<DHParameter> joints, List<double> jointValues)
        {
            throw new NotImplementedException();
        }

        public int GetStatus(List<DHParameter> joints, List<double> jointValues)
        {
            throw new NotImplementedException();
        }

        public int GetTurn(List<DHParameter> joints, List<double> jointValues)
        {
            throw new NotImplementedException();
        }

        public List<double> GetAxisValues(TransformationMatrix targetMatrix, List<DHParameter> dhParam)
        {
            throw new NotImplementedException();
        }


        public readonly SortedList<int, DHParameter> DhParameterCollection;

    }
}