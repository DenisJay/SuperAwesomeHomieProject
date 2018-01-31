using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Kinematics.Base
{
	public class DHParameter
	{

		#region FIELDS

		double _a;
		double _alpha;
		double _d;
		double _theta;

		#endregion //FIELDS

		#region PROPERTIES

		public double A { get => _a; private set => _a = value; }
		public double Alpha { get => _alpha; private set => _alpha = value; }
		public double D { get => _d; set => _d = value; }
		public double Theta { get => _theta; set => _theta = value; }

		#endregion //PROPERTIES

		public DHParameter(double a, double d, double alpha, double theta)
		{
			A = a;
			D = d;
			Alpha = alpha;
			Theta = theta;
		}
	}
}
