using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Homies.SARP.Mathematics.Transformations;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Homies.SARP.Kinematics.Common
{
    /// <summary>
    /// Denevit-Hartenberg Parameters define transformations inside a kinematic chain.
    /// Alpha is used to rotate around the current X-Axis.
    /// A is the distance in X.
    /// D is the distance in Z.
    /// Theta is used to rotate around the new Z-Axis.
    /// </summary>
    public class DHParameter
    {

        #region FIELDS

        double _a;
        double _alpha;
        double _d;
        double _theta;

		TransformationMatrix _jointTransform;

        #endregion //FIELDS

        public DHParameter(double alpha, double a, double theta, double d)
        {
            A = a;
            D = d;
            Alpha = alpha;
            Theta = theta;

			JointTransform = new TransformationMatrix();
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

			//_jointTransform.DenseMatrix = DenseMatrix.OfArray(new double[,] 
			//{
			//	{ct, -st*ca, st*sa, A*ct},
			//	{st, ct*ca, -ct*sa, A*st},
			//	{0, sa, ca, D},
			//	{0, 0, 0, 1}
			//});
		}

        #region PROPERTIES

        //TODO: Wieso sind Theta und D public settable?
		// variable achskonfiguration
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

		#endregion //PROPERTIES
	}
}
