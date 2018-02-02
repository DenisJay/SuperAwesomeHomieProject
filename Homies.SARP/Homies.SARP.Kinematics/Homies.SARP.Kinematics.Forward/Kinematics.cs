using System;
using System.CodeDom;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using Homies.SARP.Machines.BaseStructure;

namespace Homies.SARP.Kinematics.Homies.SARP.Kinematics.Forward
{
    public abstract class Kinematics
    {

        #region Constants



        #endregion

        #region Attributes


        #endregion

        #region Construct

        protected Kinematics(IReadOnlyCollection<Joint> joints)
        {
            //A kinematic with no joints is invalid.
            if (joints == null || !joints.Any())
            {
                throw new ArgumentNullException(nameof(joints));
            }

            JointCollection = new SortedList<int, Joint>();

			//TODO: watch out for the < instead of the <= sign
            for (var i = 0; i < joints.Count() - 1; i++)
            {
                JointCollection.Add(i, joints.ElementAt(i));
            }

        }

        #endregion

        #region Methods

        /// <summary>
        /// Returns the position hit from the current configuration.
        /// </summary>
        public Matrix3D GetForwardTransformationMatrix()
        {
            var zAxisBase = new Vector3D(0, 0, 1);
            var xAxisBase = new Vector3D(1, 0, 0);

            var transformGroup = new Transform3DGroup();

            for (int i = 0; i < JointCollection.Count - 1; i++)
            {
                var currentJoint = JointCollection[i];

                //Drehen um X
                var alphaRotate = new RotateTransform3D(new AxisAngleRotation3D(xAxisBase, currentJoint.DhParameter.Alpha));
                //Translation
                var translate = new TranslateTransform3D(currentJoint.DhParameter.A, 0, currentJoint.DhParameter.D);
                //Rotation around the joint-axis
                var thetaRotate = new RotateTransform3D(new AxisAngleRotation3D(zAxisBase, currentJoint.DhParameter.Theta));

                transformGroup.Children.Insert(0, alphaRotate);
                transformGroup.Children.Insert(1, translate);
                transformGroup.Children.Insert(2, thetaRotate);

                //transformGroup.Children.Add(alphaRotate);
                //transformGroup.Children.Add(translate);
                //transformGroup.Children.Add(thetaRotate);
            }

            return transformGroup.Value;
        }


        #endregion

        #region Properties

        public readonly SortedList<int, Joint> JointCollection;

        #endregion

    }

}