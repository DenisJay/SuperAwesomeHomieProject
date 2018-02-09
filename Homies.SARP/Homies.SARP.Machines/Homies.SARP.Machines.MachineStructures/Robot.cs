using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Mathematics.Transformations;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Homies.SARP.Machines.MachineStructures
{
	public class Robot
	{
		string _modelName;
		SortedList<int, Joint> _joints;

		public string ModelName
		{
			get { return _modelName; }
			set { _modelName = value; }
		}

		public SortedList<int, Joint> Joints
		{
			get { return _joints; }
			set { _joints = value; }
		}

		public TransformationMatrix TCP { get; set; }

		public Robot(string name, List<DHParameter> dhParam)
		{
			Joints = new SortedList<int, Joint>();
			ModelName = name;

			for (int i = 0; i < dhParam.Count; i++)
			{
				Joints.Add(i, new RotationalJoint(0, 360, dhParam[i]));
			}
		}
	}
}