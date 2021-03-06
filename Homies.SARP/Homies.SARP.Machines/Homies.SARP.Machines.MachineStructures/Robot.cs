﻿using Homies.SARP.Common.Extensions;
using Homies.SARP.Kinematics;
using Homies.SARP.Kinematics.Inverse;
using Homies.SARP.Machines.BaseStructure;
using Homies.SARP.Machines.Factories;
using Homies.SARP.Mathematics.Primitives;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;
using System.Collections.Generic;
using System.Linq;

namespace Homies.SARP.Machines.MachineStructures
{
	public class Robot
    {

        #region FIELDS

        string _modelName;
        SortedList<int, Joint> _joints;
        TransformationMatrix _currentTarget;
		TransformationMatrix _target;

		//TODO: implement transformations
		TransformationMatrix _joint6ToFlangeTrafo;
		
		MachineFrames _frames;
		InverseKinematics _invKin;

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
		
		private void InitializeMember()
		{
			Joint6ToFlangeTrafo = new TransformationMatrix(Transformations.GetIdentityMatrix());
			InvKin = new InverseKinematics();
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
                double radAngle = angle.DegToRad();
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
		
        private void ComputeCurrentTarget()
        {
            TransformationMatrix target = new TransformationMatrix();
			
            foreach (var joint in Joints)
            {
                target *= joint.Value.JointTransformation;
            }

			target *= Joint6ToFlangeTrafo;
            CurrentTarget = target;
        }

		public void ComputeAnglesForTargetFrame(TransformationMatrix target)
		{
			var dhs = (from joint in Joints select joint.Value.DhParameter).ToList();

			Target = target;

			InvKin.GetAngles1To3(new XPoint(
				TargetWrist.Matrix3D.OffsetX,
				TargetWrist.Matrix3D.OffsetY, 
				TargetWrist.Matrix3D.OffsetZ), dhs);

			InvKin.GetAngles4To6(target, dhs);
		}

		private TransformationMatrix ComputeWristFrame(TransformationMatrix currentTarget)
		{
			DenseMatrix addInverse = (DenseMatrix)Joints.Last().Value.JointTransformation.DenseMatrix * Joint6ToFlangeTrafo.DenseMatrix;
			DenseMatrix wrist = currentTarget.DenseMatrix * (DenseMatrix)addInverse.Inverse();

			return new TransformationMatrix(wrist);
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
            private set { _currentTarget = value; }
        }

        public TransformationMatrix CurrentWrist
        {
            get { return ComputeWristFrame(CurrentTarget); }
        }

		public TransformationMatrix Joint6ToFlangeTrafo
		{
			get { return _joint6ToFlangeTrafo; }
			set { _joint6ToFlangeTrafo = value; }
		}
		
		public TransformationMatrix Target
		{
			get { return _target; }
			private set { _target = value; }
		}

		public TransformationMatrix TargetWrist
		{
			get { return ComputeWristFrame(Target); }
		}

		public InverseKinematics InvKin
		{
			get { return _invKin; }
			set { _invKin = value; }
		}

		public MachineFrames Frames
		{
			get { return _frames; }
			set { _frames = value; }
		}

		public RobotKinematics Kinematic;

        #endregion //PROPERTIES
    }
}