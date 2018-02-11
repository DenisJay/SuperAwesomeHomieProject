using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Mathematics.Transformations;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace Homies.SARP.Machines.MachineStructures
{
    //TODO: Sollte so ein Robot nicht RobotKinematics erben/haben?
    //Wegen einer Ringabhängigkeit kann Machines aber kein Verweis auf Kinematics gegeben werden...
    public class Robot
    {
        string _modelName;
        SortedList<int, Joint> _joints;

        public Robot(string name, List<DHParameter> dhParam)
        {
            ModelName = name;
            Joints = new SortedList<int, Joint>();

            for (int i = 0; i < dhParam.Count; i++)
            {
                Joints.Add(i, new RotationalJoint(0, 360, dhParam[i]));
            }
        }

        public Robot(string name, List<Joint> joints)
        {
            ModelName = name;
            Joints = new SortedList<int, Joint>();

            for (int i = 0; i < joints.Count; i++)
            {
                Joints.Add(i, joints[i]);
            }

        }

        public string ModelName
        {
            get { return _modelName; }
            set { _modelName = value; }
        }

        public TransformationMatrix TCP { get; set; }

        public SortedList<int, Joint> Joints
        {
            get { return _joints; }
            set { _joints = value; }
        }

    }
}