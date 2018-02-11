using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace Homies.SARP.Common.Homies.SARP.Common.Extensions
{
    public static class Media3DExtensions
    {

        public static Vector3D XAxis(this Matrix3D matrix)
        {
            return new Vector3D(matrix.M11, matrix.M12, matrix.M13);
        }

        public static Vector3D YAxis(this Matrix3D matrix)
        {
            return new Vector3D(matrix.M21, matrix.M22, matrix.M23);
        }

        public static Vector3D ZAxis(this Matrix3D matrix)
        {
            return new Vector3D(matrix.M31, matrix.M32, matrix.M33);
        }

        public static Vector3D Offset(this Matrix3D matrix)
        {
            return new Vector3D(matrix.OffsetX, matrix.OffsetY, matrix.OffsetZ);
        }

    }
}
