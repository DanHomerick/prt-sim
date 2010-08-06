// Copyright (C) 2010 Duncan Hall
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

package edu.ucsc.track_builder.elevation
{
	/**
	 * PolylineEncoder
	 * @author Duncan Hall
	 * http://blog.duncanhall.net/2010/02/as3-encoded-polyline-algorithm-for-google-maps/
	 */
	
	import com.google.maps.LatLng;
	import com.google.maps.interfaces.IPolyline;
	
	public class PolylineEncoder 
	{	
		/**
		 * Converts a list of level values into a single encoded string
		 * 
		 * @param levels	The list of uint level values to encode.
		 * @return			The encoded levels.
		 */
		public static function encodeLevels (levels:Array) : String
		{
			var encoded:String = "";
			for each (var level:uint in levels) encoded += encodeUnsigned(level);
			return encoded;
		}
		
		
		
		
		/**
		 * Takes a list of <code>com.google.maps.LatLng</code> points and encodes
		 * them into a single string, for use with the <code>Polyline.fromEncoded()</code>
		 * method.
		 * 
		 * @param points	The list of points to encode.
		 * @return			The encoded string representation.
		 */
		public static function fromPoints(points:Vector.<LatLng>) : String
		{
			var encoded:String = "";
			var pLat:Number = 0;
			var pLng:Number = 0;
			var dLat:Number;
			var dLng:Number;
			var lat_e5:Number;
			var lng_e5:Number;
			
			for each(var point:LatLng in points) 
			{
				lat_e5 = Math.round(point.lat() * 1e5);
				lng_e5 = Math.round(point.lng() * 1e5);
				dLat = lat_e5 - pLat;
				dLng = lng_e5 - pLng;
				pLat = lat_e5;
				pLng = lng_e5;
				
				encoded += encodeSigned(dLat);
				encoded += encodeSigned(dLng);
			}
			
			return encoded;
		}
		
		
		
		
		/**
		 * Converts a <code>com.google.maps.interfaces.IPolyline</code> object
		 * into an encoded string representation for use with the 
		 * <code>Polyline.fromEncoded()</code> method.
		 * 
		 * @param vertices		The IPolyline object containing the points to encode.
		 * return				The encoded string representation.
		 */
		public static function fromPolyline(vertices:IPolyline) : String
		{
			var numVertices:int = vertices.getVertexCount();
			var points:Vector.<LatLng> = new Vector.<LatLng>();
			for (var i:int = 0; i < numVertices; i++) points.push(vertices.getVertex(i));
			return fromPoints(points);
		}
		
		
		
		
		/**
		 *
		 */
		private static function encodeSigned(value:Number) : String
		{
			var leftShift:Number = value << 1;
			return(encodeUnsigned(value < 0 ? ~leftShift : leftShift));
		}
		
		
		
		/**
		 *
		 */
		private static function encodeUnsigned(value:Number) : String
		{
			var encodeString:String = "";
			while (value >= 0x20) 
			{
				encodeString += String.fromCharCode((0x20 | (value & 0x1F)) + 63);
				value >>= 5;
			}
			encodeString += String.fromCharCode(value + 63);
			return encodeString;
		}

		
	}
}