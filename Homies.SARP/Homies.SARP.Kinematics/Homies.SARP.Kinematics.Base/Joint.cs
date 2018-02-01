using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Homies.SARP.Kinematics.Base
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

        #endregion

    }
}