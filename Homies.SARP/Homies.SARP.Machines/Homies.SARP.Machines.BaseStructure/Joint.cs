using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Homies.SARP.Machines.BaseStructure
{
	public abstract class Joint
	{
		DHParameter _dhParam;
		double _lowerMotionRange;
		double _upperMotionRange;
		
		public DHParameter DhParam { get => _dhParam; set => _dhParam = value; }
		public double LowerMotionRange { get => _lowerMotionRange; set => _lowerMotionRange = value; }
		public double UpperMotionRange { get => _upperMotionRange; set => _upperMotionRange = value; }
	}
}