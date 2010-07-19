package edu.ucsc.track_builder
{
	public class TrackError extends Error
	{
		public static const INSUFFICIENT_TRACK:int = 1;
		
		public function TrackError(message:String="", id:int=0)
		{
			super(message, id);
		}
		
	}
}