using Homies.SARP.Mathematics.Transformations;
using System.Windows.Media.Media3D;

namespace Homies.SARP.Machines.BaseStructure
{
    public abstract class Joint
    {
        #region Constants

        #endregion

        #region Attributes


        #endregion

        #region Construct

        protected Joint(double motionMin, double motionMax, DHParameter dhParam)
        {
            MotionMinimum = motionMin;
            MotionMaximum = motionMax;
            DhParameter = dhParam;
        }

        #endregion

        #region Methods

        #endregion

        #region Properties

        public readonly DHParameter DhParameter;

        public readonly double MotionMinimum;

        public readonly double MotionMaximum;

        public TransformationMatrix JointTransformation
        {
            get { return DhParameter.GetJointTransformation(); }
        }

        #endregion
    }
}