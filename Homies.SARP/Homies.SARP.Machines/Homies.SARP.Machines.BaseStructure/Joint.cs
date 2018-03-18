using Homies.SARP.Kinematics.Common;
using Homies.SARP.Mathematics.Transformations;

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

        /// <summary>
        /// Returns True, if <paramref name="jointValue"/> is bigger than the MotionMinimum and smaller then the MotionMaximum.
        /// </summary>
        /// <param name="jointValue"> The value to check. </param>
		protected bool JointValueInRange(double jointValue)
		{
			if (jointValue >= MotionMinimum && jointValue <= MotionMaximum)
			{
				return true;
			}
			else
			{
				return false;
			}
		}

        #endregion

        #region Properties

        public readonly DHParameter DhParameter;

        public readonly double MotionMinimum;

        public readonly double MotionMaximum;

        public TransformationMatrix JointTransformation
        {
            get { return DhParameter.JointTransform; }
        }

		public abstract double JointValue { get; set; }

		#endregion
	}
}