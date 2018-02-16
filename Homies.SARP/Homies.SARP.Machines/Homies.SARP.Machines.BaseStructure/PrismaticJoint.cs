using Homies.SARP.Kinematics.Common;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Text;
using System.Windows.Media.Media3D;

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