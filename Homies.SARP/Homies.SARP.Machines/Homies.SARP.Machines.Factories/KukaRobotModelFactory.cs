﻿using Homies.SARP.Kinematics.Common;
using System;
using System.Collections.Generic;

namespace Homies.SARP.Machines.Factories
{
	public class KukaRobotModelFactory
	{
		internal static List<DHParameter> GetDHForKR270R2700()
		{
            var parameter = new List<DHParameter>
            {
                new DHParameter(Math.PI, 0, 0, -675),
                new DHParameter(Math.PI / 2, 350, -Math.PI / 2, 0, 90),
                new DHParameter(0, 1150, 0, 0, 1.9568445888303185),
                new DHParameter(Math.PI / 2, -41, 0, -1200),
                new DHParameter(-Math.PI / 2, 0, 0, 0),
                new DHParameter(Math.PI / 2, 0, 0, -240)
            };

            return parameter;
		}
	}
}
