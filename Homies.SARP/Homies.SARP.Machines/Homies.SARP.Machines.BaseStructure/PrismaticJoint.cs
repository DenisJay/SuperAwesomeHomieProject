using Homies.SARP.Kinematics.Common;

namespace Homies.SARP.Machines.BaseStructure
{
	public class PrismaticJoint : Joint
    {

        #region Constants

        #endregion

        #region Attributes

        #endregion

        #region Construct

        //TODO: Abgleichen, ob DH Parameter min unter-, oder max überschreiten
        public PrismaticJoint(double motionMinInMillimeters, double motionMaxInMillimeters, DHParameter dhParam) : base(motionMinInMillimeters, motionMaxInMillimeters, dhParam)
        {

        }

		#endregion

		#region Methods

		#endregion

		#region Properties

		public override double JointValue
		{
			get { return this.DhParameter.D; }
			set
			{
				if (JointValueInRange(value))
				{
					this.DhParameter.D = value;
				}
			}
		}

		#endregion

	}
}