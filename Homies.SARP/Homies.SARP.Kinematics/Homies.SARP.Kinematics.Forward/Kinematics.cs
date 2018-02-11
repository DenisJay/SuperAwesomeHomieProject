using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using Homies.SARP.Common.Homies.SARP.Common.Extensions;
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

            for (var i = 0; i < joints.Count; i++)
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
            var zAxis = new Vector3D(0, 0, 1);
            var xAxis = new Vector3D(1, 0, 0);

            var transformGroup = new Transform3DGroup();

            for (int i = 0; i < JointCollection.Count; i++)
            {
                var currentJoint = JointCollection[i];

                zAxis = transformGroup.Value.ZAxis();
                zAxis.Normalize();
                
                //Translation in Z
                var zTranslate = new TranslateTransform3D(zAxis * currentJoint.DhParameter.D);
                //Rotation around the z-Axis
                var thetaRotate = new RotateTransform3D(new AxisAngleRotation3D(zAxis, currentJoint.DhParameter.Theta * 180 / Math.PI));

                transformGroup.Children.Insert(0, zTranslate);
                transformGroup.Children.Insert(1, thetaRotate);

                xAxis = transformGroup.Value.XAxis();
                xAxis.Normalize();

                //Translation in X
                var xTranslate = new TranslateTransform3D(xAxis * currentJoint.DhParameter.A);
                //Rotation around X
                var alphaRotate = new RotateTransform3D(new AxisAngleRotation3D(xAxis, currentJoint.DhParameter.Alpha * 180 / Math.PI));

                //Alpha Rotation has to be first, as the x-Axis 
                transformGroup.Children.Insert(0, alphaRotate);
                transformGroup.Children.Insert(1, xTranslate);

            }

            return transformGroup.Value;
        }

        #endregion

        #region Properties

        public readonly SortedList<int, Joint> JointCollection;

        #endregion

    }

}