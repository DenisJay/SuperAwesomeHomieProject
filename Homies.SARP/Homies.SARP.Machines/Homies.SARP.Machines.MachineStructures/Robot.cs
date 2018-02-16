using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;

namespace Homies.SARP.Machines.MachineStructures
{
	//TODO: Sollte so ein Robot nicht RobotKinematics erben/haben?
	//Wegen einer Ringabhängigkeit kann Machines aber kein Verweis auf Kinematics gegeben werden...
	public class Robot
    {

		#region FIELDS

		string _modelName;
		SortedList<int, Joint> _joints;
		TransformationMatrix _currentTarget;
		TransformationMatrix _currentWristFrame;
		#endregion //FIELDS

		#region PROPERTIES

		public string ModelName
		{
			get { return _modelName; }
			set { _modelName = value; }
		}

		public TransformationMatrix FlangeToTCP
		{
			get;
			set;
		}

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
			private set
			{
				_currentTarget = value;
			}
		}

		public TransformationMatrix CurrentWristFrame
		{
			get
			{
				ComputeCurrentWristFrame();
				return _currentWristFrame;
			}
			private set
			{
				_currentWristFrame = value;
			}
		}

		#endregion //PROPERTIES

		#region INITIALIZATION

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

		#endregion //INITIALIZATION

		#region METHODS

		private void ComputeCurrentWristFrame()
		{
			DenseMatrix wrist = (DenseMatrix)Joints.Last().Value.JointTransformation.DenseMatrix.Inverse() * CurrentTarget.DenseMatrix;

			if (CurrentWristFrame == null)
			{
				CurrentWristFrame = new TransformationMatrix(wrist);
			}
			else
			{
				CurrentWristFrame.DenseMatrix = wrist;
			}
		}

		private void ComputeCurrentTarget()
		{
			TransformationMatrix target = new TransformationMatrix();

			foreach (var joint in Joints)
			{
				target.DenseMatrix *= joint.Value.JointTransformation.DenseMatrix;
			}

			CurrentTarget = target;
		}

		#endregion //METHODS

	}
}