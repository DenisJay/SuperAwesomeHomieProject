using Homies.SARP.Mathematics.Transformations;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Homies.SARP.Machines.MachineStructures
{
	public class MachineFrames
	{

		#region FIELDS

		int _currentToolFrameNumber;
		int _currentBaseFramenumber;

		TransformationMatrix _currentToolFrame;
		TransformationMatrix _currentBaseFrame;

		List<TransformationMatrix> _toolFrames;
		List<TransformationMatrix> _baseFrames;

		#endregion

		#region PROPERTIES

		public int CurrentToolFrameNumber
		{
			get { return _currentToolFrameNumber; }
			set { _currentToolFrameNumber = value; }
		}

		public int CurrentBaseFramenumber
		{
			get { return _currentBaseFramenumber; }
			set { _currentBaseFramenumber = value; }
		}

		public TransformationMatrix CurrentToolFrame
		{
			get { return _currentToolFrame; }
			set { _currentToolFrame = value; }
		}

		public TransformationMatrix CurrentBaseFrame
		{
			get { return _currentBaseFrame; }
			set { _currentBaseFrame = value; }
		}

		public List<TransformationMatrix> ToolFrames
		{
			get { return _toolFrames; }
			set { _toolFrames = value; }
		}

		public List<TransformationMatrix> BaseFrames
		{
			get { return  _baseFrames; }
			set { _baseFrames = value; }
		}

		#endregion

		#region INITIALIZATION

		public MachineFrames()
		{
			ToolFrames = new List<TransformationMatrix>();
			BaseFrames = new List<TransformationMatrix>();
		}
		
		#endregion

		#region METHODS

		#endregion

	}
}
