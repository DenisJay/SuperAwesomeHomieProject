using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace Homies.SARP.Mathematics.Homies.SARP.Mathematics.Extensions
{

    public static class MatrixExtensions
    {
        public static Matrix3D GetRelativeMatrix(this Matrix3D from, Matrix3D to)
        {
            //An inverted matrix is equivalent to the matrix needed to get from there towards the origin.
            var matrixFrom = from;
            matrixFrom.Invert();

            //From the origin you can then go towards the desired matrix.
            var rel3DGroup = new Transform3DGroup();
            rel3DGroup.Children.Add(new MatrixTransform3D(matrixFrom));
            rel3DGroup.Children.Add(new MatrixTransform3D(to));

            return rel3DGroup.Value;
        }
    }
}
