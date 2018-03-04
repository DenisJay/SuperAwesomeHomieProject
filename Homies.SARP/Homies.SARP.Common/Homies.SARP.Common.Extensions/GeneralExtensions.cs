using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Common.Extensions
{

    public static class GeneralExtensions
    {

        #region Const

        public const double tolerance = 0.0001;

        #endregion


        /// <summary>
        /// Returns true, if the object is of the specified type.
        /// </summary>
        /// <typeparam name="T"> Type to check the object against. </typeparam>
        /// <param name="obj"> Object to be checked. </param>
        public static bool IsAFreaking<T>(this object obj)
        {
            return (obj is T);
        }

        /// <summary>
        /// Returns true, if the object is not of the specified type.
        /// </summary>
        /// <typeparam name="T"> Type to check the object against. </typeparam>
        /// <param name="obj"> Object to be checked. </param>
        public static bool IsNotAFreaking<T>(this object obj)
        {
            return !IsAFreaking<T>(obj);
        }
		
		public static bool DoublesEqual(this double[] vals1, double[] vals2)
		{
			if (vals1 == null || vals2 == null)
			{
				return false;
			}

			if (vals1.Length != vals2.Length)
			{
				return false;
			}

			for (int i = 0; i < vals1.Length; i++)
			{
				if (Math.Abs(vals1[i] - vals2[i]) > tolerance)
				{
					return false;
				}
			}

			return true;
		}

        public static bool DoubleEquals(this double val1, double val2)
        {

            if (Math.Abs(val1 - val2) < tolerance)
            {
                return true;
            }

            return false;

        }

        public static double L2Norm(this double[] values)
        {
            if (values == null || values.Length == 0)
            {
                return 0;
            }

            double squaredSum = 0;
            foreach (var value in values)
            {
                squaredSum += Math.Pow(value, 2);
            }

            return Math.Sqrt(squaredSum);
        }
    }
}
