using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Machines.BaseStructure
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


		public double Theta
		{
			get { return _theta; }
			set { _theta = value; }
		}

		public double A
		{
			get { return _a; }
			private set { _a = value; }
		}

		public double Alpha
		{
			get { return _alpha; }
			private set { _alpha = value; }
		}

		public double D
		{
			get { return _d; }
			set { _d = value; }
		}

		#endregion //PROPERTIES



		public DHParameter(double alpha, double a, double theta, double d)
		{
			A = a;
			D = d;
			Alpha = alpha;
			Theta = theta;
		}
	}
}
