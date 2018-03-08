using System;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Kinematics.Common
{
    /// <summary>
    /// Denevit-Hartenberg Parameters define transformations inside a kinematic chain.
    /// D is the distance in Z.
    /// Theta is used to rotate around the new Z-Axis.
    /// A is the distance in X.
    /// Alpha is used to rotate around the current X-Axis.
    /// </summary>
    public class DHParameter
    {

        #region FIELDS

        double _a;
        double _alpha;
        double _d;
        double _theta;

		double _thetaStandard;
		double _dStandard;

		double _angleOffset;

		TransformationMatrix _jointTransform = new TransformationMatrix();
		TransformationMatrix _jointStandardTransform = new TransformationMatrix();

        #endregion //FIELDS

        public DHParameter(double alpha, double a, double theta, double d)
        {
            A = a;
            D = d;
			_dStandard = d;
            Alpha = alpha;
            Theta = theta;
			_thetaStandard = theta;
			AngleOffset = 0;
        }

		public DHParameter(double alpha, double a, double theta, double d, double angleOffset):this(alpha, a, theta, d)
		{
			AngleOffset = angleOffset;
		}

        private void GetJointTransformation()
        {
            double ct = Math.Cos(Theta);
            double st = Math.Sin(Theta);
            double ca = Math.Cos(Alpha);
            double sa = Math.Sin(Alpha);

            _jointTransform.DenseMatrix = DenseMatrix.OfArray(new double[,]
            {
                {   ct,   -st,   0,     A},
                {ca*st, ca*ct, -sa, -D*sa},
                {sa*st, sa*ct,  ca,  D*ca},
                {    0,     0,   0,     1}
            });
        }

		private void GetStandardTransformation()
		{
			double ct = Math.Cos(_thetaStandard);
			double st = Math.Sin(_thetaStandard);
			double ca = Math.Cos(Alpha);
			double sa = Math.Sin(Alpha);

			_jointStandardTransform.DenseMatrix = DenseMatrix.OfArray(new double[,]
			{
				{   ct,   -st,   0,     A},
				{ca*st, ca*ct, -sa, -_dStandard*sa},
				{sa*st, sa*ct,  ca,  _dStandard*ca},
				{    0,     0,   0,     1}
			});
		}

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

        public TransformationMatrix JointTransform
        {
            get
            {
                GetJointTransformation();
                return _jointTransform;
            }
            private set { _jointTransform = value; }
        }

		public TransformationMatrix JointStandardTransform
		{
			get
			{
				GetStandardTransformation();
				return _jointStandardTransform;
			}
			private set { _jointStandardTransform = value; }
		}

		public double AngleOffset
		{
			get { return _angleOffset; }
			private set { _angleOffset = value; }
		}

		#endregion //PROPERTIES
	}
}
