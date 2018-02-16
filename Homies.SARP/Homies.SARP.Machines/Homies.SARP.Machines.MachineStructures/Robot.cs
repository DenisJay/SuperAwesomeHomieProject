using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;
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
		TransformationMatrix _currentTarget;
		TransformationMatrix _currentWristFrame;

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

        public TransformationMatrix FlangeToTCP { get; set; }

        public SortedList<int, Joint> Joints
        {
            get { return _joints; }
            set { _joints = value; }
        }

		public TransformationMatrix CurrentTarget
		{
			get
			{
				ComputeCurrentTarget();
				return _currentTarget;
			}
		}

		public TransformationMatrix CurrentWristFrame
		{
			get
			{
				ComputeCurrentWristFrame();
				return _currentWristFrame;
			}
		}

		private void ComputeCurrentWristFrame()
		{
			DenseMatrix wrist = (DenseMatrix)Joints.Last().Value.JointTransformation.DenseMatrix.Inverse() * CurrentTarget.DenseMatrix;

			if (_currentWristFrame == null)
			{
				_currentWristFrame = new TransformationMatrix(wrist);
			}
			else
			{
				_currentWristFrame.DenseMatrix = wrist;
            }
		}

		private void ComputeCurrentTarget()
		{
			TransformationMatrix target = new TransformationMatrix();

			foreach (var joint in Joints)
			{
				target.DenseMatrix *= joint.Value.JointTransformation.DenseMatrix;
			}

			_currentTarget = target;
		}
	}
}