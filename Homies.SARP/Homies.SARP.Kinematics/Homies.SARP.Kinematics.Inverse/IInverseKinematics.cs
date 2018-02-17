using Homies.SARP.Kinematics.Common;
using Homies.SARP.Mathematics.Transformations;
using System.Collections.Generic;

namespace Homies.SARP.Kinematics.Inverse
{
    public interface IInverseKinematics
    {
        List<double> GetAxisValues(TransformationMatrix targetMatrix, List<DHParameter> dhParam);
    }
}