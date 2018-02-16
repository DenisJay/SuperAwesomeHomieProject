using Homies.SARP.Kinematics.Common;
using Homies.SARP.Machines.BaseStructure;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Machines.Factories
{
	public static class DHParameterFactory
	{
		public static bool _factoriesInitialized = false;
		private static KukaRobotModelFactory _kukaFactory;
		private static ABBRobotModelFactory _abbFactory;

		public static List<DHParameter> GetDhParameterForRobot(RobotModels model)
		{

			if (!_factoriesInitialized)
			{
				InitializeFactories();
			}

			var parameter = new List<DHParameter>();


			//TODO: Implement other robot models
			switch (model)
			{
				case RobotModels.Kuka_KR300_R2500:
					throw new NotImplementedException();
					break;
				case RobotModels.Kuka_KR270_R2700:
					parameter = KukaRobotModelFactory.GetDHForKR270R2700();
					break;
				case RobotModels.Kuka_KR240_R2900:
					throw new NotImplementedException();
					break;
				case RobotModels.Kuka_KR210_R3100:
					throw new NotImplementedException();
					break;
				default:
					break;
			}



			return parameter;
		}

		private static void InitializeFactories()
		{
			_kukaFactory = new KukaRobotModelFactory();
			_abbFactory = new ABBRobotModelFactory();

			_factoriesInitialized = true;
		}
	}
}
