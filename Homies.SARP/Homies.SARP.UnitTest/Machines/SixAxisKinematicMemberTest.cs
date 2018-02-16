using Homies.SARP.Machines.Factories;
using Homies.SARP.Machines.MachineStructures;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.UnitTest.Machines
{
	[TestClass]
	public class SixAxisKinematicMemberTest
	{
		Robot _testRobot;


		[TestInitialize]
		public void Initialization()
		{
			_testRobot = new Robot("testRobi", DHParameterFactory.GetDhParameterForRobot(RobotModels.Kuka_KR270_R2700));
		}

		[TestMethod]
		public void TestRobotTargetFrame()
		{
			
		}

	}
}
