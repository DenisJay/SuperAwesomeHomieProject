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

		public static bool DoubleEquals(this double val1, double val2)
		{
			if (Math.Abs(val1 - val2) < 0.0001)
			{
				return true;
			}
			return false;
		}
    }
}
