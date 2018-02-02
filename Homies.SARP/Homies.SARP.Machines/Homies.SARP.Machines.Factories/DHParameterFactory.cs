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

		public static List<DHParameter> GetDhParameterForRobot(RobotModel model)
		{

			if (!_factoriesInitialized)
			{
				InitializeFactories();
			}

			var parameter = new List<DHParameter>();

			switch (model)
			{
				case RobotModel.Kuka_KR300_R2500:
					parameter = _kukaFactory.GetDhForKR300R2500();
					break;
				case RobotModel.Kuka_KR270_R2700:
					parameter = _kukaFactory.GetDhForKR270R2700();
					break;
				case RobotModel.Kuka_KR240_R2900:
					parameter = _kukaFactory.GetDhForKR240R2900();
					break;
				case RobotModel.Kuka_KR210_R3100:
					parameter = _kukaFactory.GetDhForKR210R3100();
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
