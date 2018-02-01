﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Homies.SARP.Kinematics.Base;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Forward
{
    public class RobotKinematics : Kinematics
    {
        public RobotKinematics(IReadOnlyCollection<Joint> joints) : base(joints)
        {

        }
    }
}
