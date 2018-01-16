using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net.Mime;
using System.Reflection;
using laszip.net;
//using LibLAS;

namespace lidar_orto_photo
{
    internal class Program
    {
	    
	    public static readonly String RESOURCE_DIRECTORY_PATH = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + "\\resources\\";
	    public static readonly String temp_file_name = "GK_470_97.laz";
	    
	    
	    
        struct Point3D
		{
			public double X;
			public double Y;
			public double Z;
		}

		public static void test3()
		{
			WriteLaz(); // Write a file
			ReadLaz();  // Read it back
		}

		static string FileName = Path.GetTempPath() + "Test.laz";

		static void ReadLaz()
		{
			var lazReader = new laszip_dll();
			var compressed = true;
			lazReader.laszip_open_reader(FileName, ref compressed);
			var numberOfPoints = lazReader.header.number_of_point_records;

			// Check some header values
			Debug.WriteLine(lazReader.header.min_x);
			Debug.WriteLine(lazReader.header.min_y);
			Debug.WriteLine(lazReader.header.min_z);
			Debug.WriteLine(lazReader.header.max_x);
			Debug.WriteLine(lazReader.header.max_y);
			Debug.WriteLine(lazReader.header.max_z);

			int classification = 0;
			var point = new Point3D();
			var coordArray = new double[3];

			// Loop through number of points indicated
			for (int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
			{
				// Read the point
				lazReader.laszip_read_point();
				
				// Get precision coordinates
				lazReader.laszip_get_coordinates(coordArray);
//				lazReader.
				point.X = coordArray[0];
				point.Y = coordArray[1];
				point.Z = coordArray[2];
				
				// Get classification value
				classification = lazReader.point.classification;
			}

			// Close the reader
			lazReader.laszip_close_reader();
		}

		private static void WriteLaz()
		{
			// --- Write Example
			var point = new Point3D();
			var points = new List<Point3D>();

			point.X = 1000.0;
			point.Y = 2000.0;
			point.Z = 100.0;
			
			points.Add(point);

			point.X = 5000.0;
			point.Y = 6000.0;
			point.Z = 200.0;
			points.Add(point);

			var lazWriter = new laszip_dll();
			var err = lazWriter.laszip_clean();
			if (err == 0)
			{
				// Number of point records needs to be set
				lazWriter.header.number_of_point_records = (uint)points.Count;

				// Header Min/Max needs to be set to extents of points
				lazWriter.header.min_x = points[0].X; // LL Point
				lazWriter.header.min_y = points[0].Y;
				lazWriter.header.min_z = points[0].Z;
				lazWriter.header.max_x = points[1].X; // UR Point
				lazWriter.header.max_y = points[1].Y;
				lazWriter.header.max_z = points[1].Z;

				// Open the writer and test for errors
				err = lazWriter.laszip_open_writer(FileName, true);
				if (err == 0)
				{
					double[] coordArray = new double[3];
					foreach (var p in points)
					{
						coordArray[0] = p.X;
						coordArray[1] = p.Y;
						coordArray[2] = p.Z;

						// Set the coordinates in the lazWriter object
						lazWriter.laszip_set_coordinates(coordArray);

						// Set the classification to ground
						lazWriter.point.classification = 2;

//						new laszip_point().
						// Write the point to the file
						err = lazWriter.laszip_write_point();
						if (err != 0) break;
					}

					// Close the writer to release the file (OS lock)
					err = lazWriter.laszip_close_writer();
					lazWriter = null;
				}
			}

			if (err != 0)
			{
				// Show last error that occurred
				Debug.WriteLine(lazWriter.laszip_get_error());
			}
			// --- Upon completion, file should be 389 bytes
}

	    private static void runLasViewer()
	    {
		    ProcessStartInfo start = new ProcessStartInfo();
		    start.Arguments = "-i \"" + RESOURCE_DIRECTORY_PATH + temp_file_name + "\""; 
		    start.FileName = RESOURCE_DIRECTORY_PATH + "lasview";
		    start.WindowStyle = ProcessWindowStyle.Normal;
		    start.CreateNoWindow = false;
			Process.Start(start);
		    
	    }
	    
        public static void Main(string[] args)
        {
	        Console.WriteLine("start");
            
            Console.WriteLine(RESOURCE_DIRECTORY_PATH);
//	        runLasViewer();
	        
	        
	        Console.WriteLine("end");
        }//end main

	    public static void test()
	    {
		    var totalPointCount = 0L;
		    var filename = Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName + "\\resources\\GK_470_97.laz";
//            var filename = "C:\\Users\\Matej\\IdeaProjects\\nrg-seminar\\src\\GK_470_97.laz";
		    File.OpenRead(filename);
		    var info = LASZip.Parser.ReadInfo(filename);
		    totalPointCount += info.Count;
		    var a = true;
		    Console.WriteLine($"{Path.GetFileName(filename),-40}{info.Count,20:N0}");
		    foreach (var ps in LASZip.Parser.ReadPoints(filename, 1))
		    {
//                    bounds2.ExtendBy(new Box3d(ps.Positions));
//                    Console.WriteLine($"  chunk {ps.Count,20:N0}");
//                laszip_dll.laszip_create().point
//	            if (a)
//	            {
//		            a = false;
//		            if (ps.Classifications[0] != 4) Console.WriteLine(ps.Classifications[0]);
			    if (ps.Colors[0][0] != 0 )Console.WriteLine(ps.Colors[0]);
//	            if (ps.Positions[0] != 4)Console.WriteLine(ps.Positions[0]);
//		            Console.WriteLine(ps.Classifications);
//		            Console.WriteLine(ps.Classifications);
			    
			    //17,21,25 biti xt
//	            }
		    }
            
		    Console.WriteLine($"total point count: {totalPointCount:N0}");
            
//	        
		    
	    }
    } //end class
}