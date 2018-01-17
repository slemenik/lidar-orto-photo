using System;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Runtime.CompilerServices;
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
//	    private static readonly String temp_file_name = "GK_470_97.laz";	   
        
	    private static readonly string ARSO_LIDAR_URL = "http://gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_470_97.laz";
	    private static readonly string ARSO_ORTOPHOTO_URL = "http://gis.arso.gov.si/arcgis/rest/services/DOF_2016/MapServer/export";
	    private static readonly int ORTO_PHOTO_IMG_SIZE = 2000;//TODO - change so it works with different sizes

	    private static int bottomLeftX;
	    private static int bottomLeftY;

	    static void setParameters(string fileName)
	    {

		    bottomLeftX = Int32.Parse(fileName.Split('_')[1]) * 1000;
		    bottomLeftY = Int32.Parse(fileName.Split('_')[2]) * 1000;
		    
	    }
	    
	    
		static void ReadLaz(string fileName, string imageName)
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
				
				
				var img = new Bitmap(ResourceDirectoryPath + imageName, true);
//				int[] pixels = buff.getData();
//				int height = image.getHeight();
//				int width = image.getWidth();

	
				lazWriter.header.number_of_point_records = numberOfPoints;
				lazWriter.header = lazReader.header;


				var newFileName = Path.GetFileNameWithoutExtension(fileName) + "_new.laz";
				err = lazWriter.laszip_open_writer(ResourceDirectoryPath + newFileName, true);
				if (err == 0)
				{
					Console.Write("[{0:h:mm:ss}] Reading and writing points ...", DateTime.Now);
					for (var pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
					{
						lazReader.laszip_read_point();

						// Get precision coordinates
						lazReader.laszip_get_coordinates(coordArray);			
//						lazWriter.laszip_set_coordinates(coordArray);

						lazWriter.point = lazReader.point;
						int[] pxCoordinates = findClosestPxCoordinates(coordArray[0], coordArray[1]);
						int i = (pxCoordinates[0]-bottomLeftX)*2;
						int j = img.Height-1-((pxCoordinates[1]-bottomLeftY)*2);//j index of image goes from top to bottom
						
//						var rnd = new Random();				
//						var r = rnd.Next(0, 255);
//						var g = rnd.Next(0, 255);
//						var b = rnd.Next(0, 255);
//						
//						Console.WriteLine(pxCoordinates[0]);
//						Console.WriteLine(i);
						Color color = img.GetPixel(i,j); //binary int value
//						Color color = new Color(rgb);
//						Console.WriteLine("{0},{1},{2}", color.R, color.G, color.B);
//						break;
//						int[] rgbArray = new int[]{color.getRed(), color.getGreen(), color.getBlue()};
						
						lazWriter.point.rgb = new [] {(ushort) (color.R << 8), (ushort) (color.G << 8),(ushort) (color.B << 8), (ushort) 0};

						var waveformPacket = lazReader.point.wave_packet;

						var xt = 123;
						var yt = 2444;
						var zt = 132122;
////
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
//							Console.WriteLine(lazReader.point.rgb[0]);
//							Console.WriteLine(lazReader.point.rgb[1]);
//							Console.WriteLine(lazReader.point.rgb[2]);
//							Console.WriteLine(lazReader.point.rgb[3]);
//							
//							Console.WriteLine((int) lazReader.point.rgb[0] >> 8);
//							Console.WriteLine((int) lazReader.point.rgb[1] >> 8);
//							Console.WriteLine((int) lazReader.point.rgb[2] >> 8);
//							Console.WriteLine((int) lazReader.point.rgb[3] >> 8);
//							Console.WriteLine(lazReader.point.rgb[3]);
							
//							Console.WriteLine(lazReader.point.wave_packet);
							var a = lazReader.point.wave_packet;
//							for (int i = 0; i < a.Length; i++)
//							{
//								Console.Write(a[i] + "  ");
//							}
//							break;
							classification++;
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
		    Console.Write("[{0:h:mm:ss}] Opening lasview.exe...", DateTime.Now);
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
		    Console.Write("[{0:h:mm:ss}] Converting to LAS 1.3 ...", DateTime.Now);
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
	        Console.WriteLine("[{0:h:mm:ss}] Start program. ", DateTime.Now);
	        var fileName = "GK_470_97";
	        var imageName = "export2.png";

	        
	        
	        setParameters(fileName);
////	        runLas2las(fileName + ".laz");
	        ReadLaz(fileName + "_1.laz", imageName);
//	        runLasview(fileName + "_1_new.laz");
//	        var a = isPointOutOfBounds(new Point(470000.0, 97000.0), 470000, 97000);
//	        Console.WriteLine(bottomLeftX);
//	        Console.WriteLine(bottomLeftY);
//	        Button1_Click(imageName);
	        
	        Console.WriteLine("[{0:h:mm:ss}] End program.", DateTime.Now);
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
		    
		    double maxX = bottomLeftX + (ORTO_PHOTO_IMG_SIZE-1);
		    double maxY = bottomLeftY + (ORTO_PHOTO_IMG_SIZE-1);

		    return p.X > maxX || p.Y > maxY;
	    }
	    
    } //end class
}