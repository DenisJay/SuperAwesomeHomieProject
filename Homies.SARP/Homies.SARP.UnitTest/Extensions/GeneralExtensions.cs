using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.UnitTest.Extensions
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
            return !(obj is T);
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

    }
}
