using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Mathematics.Transformations;
using Homies.SARP.Kinematics.Common;
using System;
using Homies.SARP.Kinematics;
using Homies.SARP.Machines.Factories;

namespace Homies.SARP.Machines.MachineStructures
{
    public class Robot
    {

        #region FIELDS

        string _modelName;
        SortedList<int, Joint> _joints;
        TransformationMatrix _currentTarget;
        TransformationMatrix _currentWristFrame;

		//TODO: implement transformations
		TransformationMatrix _joint6ToFlangeTrafo;
		List<TransformationMatrix> _tcpTrafos;
		int _currentTCP;

        #endregion //FIELDS

        #region INITIALIZATION

		public Robot(string name, RobotManufacturer robotBrand, RobotModels robotModel)
		{
			ModelName = name;
			Joints = new SortedList<int, Joint>();

			var dhParams = DHParameterFactory.GetDhParameterForRobot(robotModel);

			for (int i = 0; i < dhParams.Count; i++)
			{
				Joints.Add(i, new RotationalJoint(0, 360, dhParams[i]));
			}

			Kinematic = new RobotKinematics(dhParams);
			InitializeMember();
			Joint6ToFlangeTrafo = RobotBaseDataProvider.GetJoint6ToFlangeTransformation(robotBrand);
		}

        public Robot(string name, List<DHParameter> dhParams)
        {
            ModelName = name;
            Joints = new SortedList<int, Joint>();

            for (int i = 0; i < dhParams.Count; i++)
            {
                Joints.Add(i, new RotationalJoint(0, 360, dhParams[i]));
            }

            Kinematic = new RobotKinematics(dhParams);
			InitializeMember();
        }

		public Robot(string name, List<Joint> joints)
        {
            ModelName = name;
            Joints = new SortedList<int, Joint>();

            for (int i = 0; i < joints.Count; i++)
            {
                Joints.Add(i, joints[i]);
            }

            var dhParams = (List<DHParameter>) (from joint in Joints select joint.Value.DhParameter);
            Kinematic = new RobotKinematics(dhParams);
			InitializeMember();
		}
		
		private void InitializeMember()
		{
			TcpTrafos = new List<TransformationMatrix>();
			Joint6ToFlangeTrafo = new TransformationMatrix(Transformations.GetIdentityMatrix());
		}

		public void SetJoint6ToFlangeTcpTrafo(RobotManufacturer roboManufacturer)
		{
			Joint6ToFlangeTrafo = RobotBaseDataProvider.GetJoint6ToFlangeTransformation(roboManufacturer);
		}
		
		#endregion //INITIALIZATION

		#region METHODS

		public void SetAnglesInDegree(List<double> degreeAngles)
        {
            if (degreeAngles == null || degreeAngles.Count < 1 || degreeAngles.Count > Joints.Count)
            {
                return;
            }

            foreach (var angle in degreeAngles)
            {
                double radAngle = angle * Math.PI / 180;
                int index = degreeAngles.IndexOf(angle);
                Joints[index].JointValue = radAngle;
            }
        }

        public void SetAnglesInRadian(List<double> radAngles)
        {
            if (radAngles == null || radAngles.Count < 1 || radAngles.Count > Joints.Count)
            {
                return;
            }

            foreach (var angle in radAngles)
            {
                Joints[radAngles.IndexOf(angle)].JointValue = angle;
            }
        }

        private void ComputeCurrentWristFrame()
        {
			DenseMatrix addInverse = (DenseMatrix)Joints.Last().Value.JointTransformation.DenseMatrix * Joint6ToFlangeTrafo.DenseMatrix;


			DenseMatrix wrist = CurrentTarget.DenseMatrix * (DenseMatrix)addInverse.Inverse();

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
                target *= joint.Value.JointTransformation;
            }

			target *= Joint6ToFlangeTrafo;

			if (CurrentTCP > 0 && TcpTrafos.Count >= CurrentTCP)
			{
				target *= TcpTrafos[CurrentTCP];
			}

            CurrentTarget = target;
        }

        #endregion //METHODS

        #region PROPERTIES

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

		public TransformationMatrix Joint6ToFlangeTrafo
		{
			get { return _joint6ToFlangeTrafo; }
			set { _joint6ToFlangeTrafo = value; }
		}

		public List<TransformationMatrix> TcpTrafos
		{
			get { return _tcpTrafos; }
			set { _tcpTrafos = value; }
		}

		public int CurrentTCP
		{
			get { return _currentTCP; }
			set { _currentTCP = value; }
		}

		public RobotKinematics Kinematic;

        #endregion //PROPERTIES        
    }
}