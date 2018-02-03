using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Homies.SARP.Machines.BaseStructure;

namespace Homies.SARP.Machines.Factories
{
	public class KukaRobotModelFactory
	{
		internal static List<DHParameter> GetDHForKR270R2700()
		{
			var parameter = new List<DHParameter>();

			parameter.Add(new DHParameter(Math.PI, 0, 0, -675));
			parameter.Add(new DHParameter(Math.PI / 2, 350, -Math.PI / 2, 0));
			parameter.Add(new DHParameter(0, 1150, 0, 0));
			parameter.Add(new DHParameter(Math.PI / 2, -41, 0, -1200));
			parameter.Add(new DHParameter(-Math.PI / 2, 0, 0, 0));
			parameter.Add(new DHParameter(Math.PI / 2, 0, 0, -240));

			return parameter;
		}
	}
}
