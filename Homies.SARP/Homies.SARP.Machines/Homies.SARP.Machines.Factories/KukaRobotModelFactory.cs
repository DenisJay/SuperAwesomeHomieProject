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
		internal List<DHParameter> GetDhForKR300R2500()
		{
			throw new NotImplementedException();
		}

		internal List<DHParameter> GetDhForKR270R2700()
		{
			var dhparam = new List<DHParameter>();

			double l1, l2, l3, l4, l5;

			l1 = 675;
			l2 = 350;
			l3 = 1150;
			l4 = 41;
			l5 = 1200;
			//TODO: need to fix this up
			dhparam.Add(new DHParameter(Math.PI, 0,	0, -l1));
			dhparam.Add(new DHParameter(Math.PI / 2, l2, -Math.PI / 2, 0));
			dhparam.Add(new DHParameter(0, l3, Math.PI, 0));
			dhparam.Add(new DHParameter(Math.PI / 2, 0, 0, 0));
			//dhparam.Add(new DHParameter(-Math.PI / 2, 0, 0, -l5));
			//dhparam.Add(new DHParameter(Math.PI / 2, 0, 0, 0));

			return dhparam;
		}

		internal List<DHParameter> GetDhForKR240R2900()
		{
			throw new NotImplementedException();
		}

		internal List<DHParameter> GetDhForKR210R3100()
		{
			throw new NotImplementedException();
		}
	}
}
