using System;
using System.Diagnostics;
using System.IO;
using laszip.net;


namespace lidar_orto_photo
{
    internal static class Program
    {
	    private static readonly String ResourceDirectoryPath = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + "\\resources\\";
//	    private static readonly String temp_file_name = "GK_470_97.laz";	   
        

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
	
				int classification = 0;
//				var coordArray = new double[3];
	
				lazWriter.header.number_of_point_records = n~umberOfPoints;

				lazWriter.header = lazReader.header;


				var newFileName = Path.GetFileNameWithoutExtension(fileName) + "_new.laz";
				err = lazWriter.laszip_open_writer(ResourceDirectoryPath + newFileName, true);
				if (err == 0)
				{
					for (var pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
					{
						lazReader.laszip_read_point();

						// Get precision coordinates
//						lazReader.laszip_get_coordinates(coordArray);			
//						lazWriter.laszip_set_coordinates(coordArray);

						lazWriter.point = lazReader.point;
						
						var rnd = new Random();				
						var r = rnd.Next(0, 255);
						var g = rnd.Next(0, 255);
						var b = rnd.Next(0, 255);
//						
						lazWriter.point.rgb = new [] {(ushort) (r << 8), (ushort) (g << 8),(ushort) (b << 8), (ushort) 0};

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
							
							Console.WriteLine(lazReader.point.wave_packet);
							var a = lazReader.point.wave_packet;
							for (int i = 0; i < a.Length; i++)
							{
								Console.Write(a[i] + "  ");
							}
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
		}

		

	    private static void runLasview(string fileName)
	    {
		    Console.Write("[{0:h:mm:ss}] Opening lasview.exe...");
		    var start = new ProcessStartInfo
		    {
			    Arguments = "-i \"" + ResourceDirectoryPath + fileName + "\"",
			    FileName = ResourceDirectoryPath + "lasview",
			    WindowStyle = ProcessWindowStyle.Normal,
			    CreateNoWindow = false
		    };

		    var process = Process.Start(start);
		    process.WaitForExit();
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
		    
//		    var process = Process.Start(ResourceDirectoryPath+"las2las", 
//			    "-i \"" + ResourceDirectoryPath+ temp_file_name
//			    +"\" -set_point_type 5 -set_version 1.3 -odir \""
//			    +ResourceDirectoryPath.Substring(0,ResourceDirectoryPath.Length-1) + "\" -o \"plis.laz\"");
		    process.WaitForExit();
		    Console.WriteLine("[DONE]");
	    }
	    
        public static void Main()
        {
	        Console.WriteLine("[{0:h:mm:ss}] Start program. ", DateTime.Now);
	        var fileName = "GK_470_97";
	        
//	        runLas2las(fileName + ".laz");
//	        ReadLaz(fileName + "_1.laz");
//	        runLasview(fileName + "_1_new.laz");
	        
	        Console.WriteLine("[{0:h:mm:ss}] End program.", DateTime.Now);
        }//end main
	    
    } //end class
}