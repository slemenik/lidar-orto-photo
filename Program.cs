using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Net;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Web.UI.DataVisualization.Charting;
using System.Windows.Forms;
using Aardvark.Base;
using laszip.net;
using System.Windows;
using Point = System.Windows.Point;


namespace lidar_orto_photo
{
    internal class Program
    {
	    private static readonly string ResourceDirectoryPath = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + "\\resources\\";	   
        
	    private static readonly string ARSO_LIDAR_URL = "http://gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_470_97.laz";
	    private const int OrtoPhotoImgSize = 2000;

	    private static int _bottomLeftX;
	    private static int _bottomLeftY;

	    static void setParameters(string fileName)
	    {

		    Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-US");
		    _bottomLeftX = Int32.Parse(fileName.Split('_')[1]) * 1000;
		    _bottomLeftY = Int32.Parse(fileName.Split('_')[2]) * 1000;
		    
	    }
	    
	    
		static void ReadLaz(string fileName)
		{
			var lazReader = new laszip_dll();
			var lazWriter = new laszip_dll();
			var err = lazWriter.laszip_clean();
			var compressed = true;
			if (err == 0)
			{
				var filePath = ResourceDirectoryPath + fileName;
				lazReader.laszip_open_reader(filePath, ref compressed);
				var numberOfPoints = lazReader.header.number_of_point_records;
	
				// Check some header values
//				Console.WriteLine(lazReader.header.min_x);
//				Console.WriteLine(lazReader.header.min_y);
//				Console.WriteLine(lazReader.header.min_z);
//				Console.WriteLine(lazReader.header.max_x);
//				Console.WriteLine(lazReader.header.max_y);
//				Console.WriteLine(lazReader.header.max_z);
//				Console.WriteLine("format: " + lazReader.header.point_data_format);
	
				var classification = 0;
				var coordArray = new double[3];

				var img = getOrthophotoImg();
//				var img = new Bitmap(ResourceDirectoryPath + imageName, true);

	
				lazWriter.header.number_of_point_records = numberOfPoints;
				lazWriter.header = lazReader.header;


				var newFileName = Path.GetFileNameWithoutExtension(fileName) + "_new.laz";
				err = lazWriter.laszip_open_writer(ResourceDirectoryPath + newFileName, true);
				if (err == 0)
				{
					Console.Write("[{0:hh:mm:ss}] Reading and writing points ...", DateTime.Now);
					for (var pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
					{
						lazReader.laszip_read_point();

						// Get precision coordinates
						lazReader.laszip_get_coordinates(coordArray);			
//						lazWriter.laszip_set_coordinates(coordArray);

						lazWriter.point = lazReader.point;
						int[] pxCoordinates = findClosestPxCoordinates(coordArray[0], coordArray[1]);
						int i = (pxCoordinates[0]-_bottomLeftX)*2;
						int j = img.Height-1-((pxCoordinates[1]-_bottomLeftY)*2);//j index of image goes from top to bottom
												
						Color color = img.GetPixel(i,j); //binary int value						
						lazWriter.point.rgb = new [] {(ushort) (color.R << 8), (ushort) (color.G << 8),(ushort) (color.B << 8), (ushort) 0};

						var waveformPacket = lazReader.point.wave_packet;

						var xt = 123;
						var yt = 2444;
						var zt = 132122;

						waveformPacket[17] = (byte)(xt >> 24);
						waveformPacket[18] = (byte)(xt >> 16);
						waveformPacket[19] = (byte)(xt >> 8);
						waveformPacket[20] = (byte) xt;
						
						waveformPacket[21] = (byte)(yt >> 24);
						waveformPacket[22] = (byte)(yt >> 16);
						waveformPacket[23] = (byte)(yt >> 8);
						waveformPacket[24] = (byte) yt;
						
						waveformPacket[25] = (byte)(zt >> 24);
						waveformPacket[26] = (byte)(zt >> 16);
						waveformPacket[27] = (byte)(zt >> 8);
						waveformPacket[28] = (byte) zt;
						
						if (classification == 0)
						{
//							
						}
//						
						err = lazWriter.laszip_write_point();
						if (err != 0) break;
						
					}
				}
				// Close the reader
				lazReader.laszip_close_reader();
			}

			if (err != 0)
			{
				// Show last error that occurred
				Debug.WriteLine(lazWriter.laszip_get_error());
			}
			
			Console.WriteLine("[DONE]");
			
		}//end function

		

	    private static void runLasview(string fileName)
	    {
		    Console.Write("[{0:hh:mm:ss}] Opening lasview.exe...", DateTime.Now);
		    var start = new ProcessStartInfo
		    {
			    Arguments = "-i \"" + ResourceDirectoryPath + fileName + "\"",
			    FileName = ResourceDirectoryPath + "lasview",
			    WindowStyle = ProcessWindowStyle.Normal,
			    CreateNoWindow = false
		    };

		    Process.Start(start);
		    Console.WriteLine("[DONE]");
	    }

	    private static void runLas2las(string fileName)
	    {
		    Console.Write("[{0:hh:mm:ss}] Converting to LAS 1.3 ...", DateTime.Now);
		    var start = new ProcessStartInfo
		    {
			    Arguments = "-i \"" + ResourceDirectoryPath + fileName + "\" -set_point_type 5 -set_version 1.3 -olaz",
			    FileName = ResourceDirectoryPath + "las2las",
			    WindowStyle = ProcessWindowStyle.Hidden,
			    CreateNoWindow = false
		    };
		    var process = Process.Start(start);
		    process.WaitForExit();
		    Console.WriteLine("[DONE]");
	    }
	    
        public static void Main()
        {
	        Console.WriteLine("[{0:hh:mm:ss}] Start program. ", DateTime.Now);
	        var fileName = "GK_470_97";
	        
	        
	        setParameters(fileName);
////	        runLas2las(fileName + ".laz");
	        ReadLaz(fileName + "_1.laz");
	        runLasview(fileName + "_1_new.laz");
//	        var a = isPointOutOfBounds(new Point(470000.0, 97000.0), 470000, 97000);
//	        Console.WriteLine(bottomLeftX);
//	        Console.WriteLine(bottomLeftY);
//	        Button1_Click(imageName);
//	        Bitmap v = getOrthophotoImg();
//	        v.Save(ResourceDirectoryPath + "kek.png");
	        
	        Console.WriteLine("[{0:hh:mm:ss}] End program.", DateTime.Now);
        }//end main
	    
	    private static int[] findClosestPxCoordinates(double x, double y){

		    var decimalPartX = x - Math.Floor(x);
		    var decimalPartY = y - Math.Floor(y);
		    x = decimalPartX >= 0 && decimalPartX < 0.5 ? (int) x : (int) x + 0.5; //0.0...0.49 -> 0.0	    
		    y = decimalPartY >= 0 && decimalPartY < 0.5 ? (int) y : (int) y + 0.5; //0.5...0.99 -> 0.5
		    
		    var p = new Point(x, y);
		    var upperLeft = new Point((int)x,(int)y+0.5);
		    var upperRight = new Point((int)x+0.5,(int)y+0.5);
		    var bottomLeft = new Point((int)x,(int)y);
		    var bottomRight = new Point((int)x+0.5,(int)y);

		    //leftBottom is never out of bounds
		    var closestPoint = bottomLeft;
		    var minDistance = (p - bottomLeft).Length;

		    var points = new []{upperLeft, upperRight, bottomRight};
		    foreach (var currPoint in points)
		    {
			    if (isPointOutOfBounds(currPoint)) continue;
			    var currDistance = (p - currPoint).Length;
			    if (currDistance < minDistance)
			    {
				    closestPoint = currPoint;
				    minDistance = currDistance;
			    }
			    
		    }
		    return new []{(int)closestPoint.X, (int)closestPoint.Y};
	    }
	    
	    private static bool isPointOutOfBounds(Point p)
	    {
		    
		    double maxX = _bottomLeftX + (OrtoPhotoImgSize-1);
		    double maxY = _bottomLeftY + (OrtoPhotoImgSize-1);

		    return p.X > maxX || p.Y > maxY;
	    }

	    public void normalEstimation()
	    {
		    List<Point3D> normals = new List<Point3D>();//List<Vector3d> Normals = new List<Vector3d>();Form an empty list of normal vectors
		    //* Point3dList PCList = new Point3dList();
		    //* PCList.AddRange(x);
		    //* double Dev = MD;Define deviation as a double
		    //* foreach (Point3d point in PCList) {For each point as Point3d in the point cloud
		    //    * 		dynamic Neighbors = PCList.FindAll(V => V.DistanceTo(point) < D);find neighbors
		    //;fit a plane to neighbors
		    //Get the normal of this plane and put it out as the normal of the point
		    //form a vector from the vantage point VP to point=VP-point and call it dir
		    //    * 		plane NP = default(Plane);
		    //    * 		Plane.FitPlaneToPoints(Neighbors, NP, Dev)
		    //    * 		if (NP.Normal * (VP - point) > 0) { if this normal.dir>0 then
		    //	    * 			Normals.Add(NP.Normal);Add the normal to the list of normals
		    //	    * 		} else {
		    //	    * 			Normals.Add(-NP.Normal);Add –normal to the list of normals
		    //	    * 		}
		    //    * }
		    //* A = Normals;
		    //* B = PCList.FindAll(VT => VT.DistanceTo(x(654)) < D); 
	    }

	    private static Bitmap getOrthophotoImg()
	    {
		    double minX = _bottomLeftX;
		    double minY = _bottomLeftY;
		    double maxX = minX + 999.999999999;
		    double maxY = minY + 999.999999999;
		    
			Console.Write("[{0:hh:mm:ss}] Downloading image...", DateTime.Now);
		    WebRequest request = WebRequest.Create("http://gis.arso.gov.si/arcgis/rest/services/DOF_2016/MapServer/export" +
		                                           $"?bbox={minX}%2C{minY}%2C{maxX}%2C{maxY}&bboxSR=&layers=&layerDefs=" +
		                                           $"&size={OrtoPhotoImgSize}%2C{OrtoPhotoImgSize}&imageSR=&format=png" +
		                                           $"&transparent=false&dpi=&time=&layerTimeOptions=" +
		                                           "&dynamicLayers=&gdbVersion=&mapScale=&f=image");
		    WebResponse response = request.GetResponse();
		    Stream responseStream = response.GetResponseStream();
		    Console.WriteLine("[DONE]");
		    return new Bitmap(responseStream);
	    }
	    
    } //end class
}