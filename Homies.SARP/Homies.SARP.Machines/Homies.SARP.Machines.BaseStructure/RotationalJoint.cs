using Homies.SARP.Kinematics.Common;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Homies.SARP.Machines.BaseStructure
{
    public class RotationalJoint : Joint
    {
        #region Constants

        #endregion

        #region Attributes

        #endregion

        #region Construct

        //TODO: Abgleichen, ob DH Parameter min unter-, oder max überschreiten
        public RotationalJoint(double motionMinInRad, double motionMaxInRad, DHParameter dhParam) : base(motionMinInRad, motionMaxInRad, dhParam)
        {

        }

		#endregion

		#region Methods

		#endregion

		#region Properties
		public override double JointValue
		{
			get { return this.DhParameter.Theta; }
			set
			{
				//if (JointValueInRange(value))
				//{
					this.DhParameter.Theta = value;
				//}
			}
		}
		#endregion
	}
}