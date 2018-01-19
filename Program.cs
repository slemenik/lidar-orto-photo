﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Net;
 using System.Runtime.InteropServices;
 using System.Runtime.Remoting.Messaging;
using System.Threading;
using System.Web.UI.DataVisualization.Charting;
using Accord.Collections;
using Accord.IO;
using Accord.Math;
 using Accord.Math.Decompositions;
 using laszip.net;
using RestSharp;
using RestSharp.Deserializers;
using Point = System.Windows.Point;


namespace lidar_orto_photo
{
	internal class Program
    {
	    private static readonly string ResourceDirectoryPath = Directory.GetParent(Directory.GetCurrentDirectory()).Parent?.FullName + "\\resources\\";	   
	    private static readonly int[] SlovenianMapBounds = {374,  30,  624,  194}; //minx,miny,maxx,maxy in thousand, manualy set based on ARSO website
	    private const int OrtoPhotoImgSize = 2000;
	    private static bool IncludeNormals = false;

	    private static int _bottomLeftX;
	    private static int _bottomLeftY;
	    private static int _addedBlocs;

	    //sets global parameters
	    //param example: "gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_462_104.laz"
	    private static void SetParameters(string url)
	    {		
		    Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-US");
		    var fileName = Path.GetFileNameWithoutExtension(url);
		    _bottomLeftX = int.Parse(fileName.Split('_')[1]) * 1000;
		    _bottomLeftY = int.Parse(fileName.Split('_')[2]) * 1000;
	    }

		//reads LAZ, builds KD tree, reads LAZ again and sets color & normal and writes
	    private static void ReadWriteLaz()
		{
			var lazReader = new laszip_dll();		
			var compressed = true;		
			var filePath = ResourceDirectoryPath + "laz13.laz";
			
			lazReader.laszip_open_reader(filePath, ref compressed);
			var numberOfPoints = lazReader.header.number_of_point_records;
			var coordArray = new double[3];
			var kdTree = new KDTree(3);
			if (IncludeNormals)
			{								
				Console.Write("[{0:hh:mm:ss}] Reading LAZ and building KD tree...", DateTime.Now);
				for (var pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
				{
					lazReader.laszip_read_point();
					lazReader.laszip_get_coordinates(coordArray);

					kdTree.Add(coordArray);
				}
				Console.WriteLine("[DONE] ");
			}		
			var img = GetOrthophotoImg();
		
			Console.Write("[{0:hh:mm:ss}] Reading and writing LAZ...", DateTime.Now);
			lazReader.laszip_seek_point(0L);//read from the beginning again
			lazReader.laszip_open_reader(filePath, ref compressed);
		
			var lazWriter = new laszip_dll();              
			lazWriter.header = lazReader.header;
			lazWriter.laszip_open_writer(ResourceDirectoryPath + "SloveniaLidarRGB" +_addedBlocs + ".laz", true);
					
			for (var pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
			{
				lazReader.laszip_read_point();
				lazReader.laszip_get_coordinates(coordArray);
				lazWriter.point = lazReader.point;
				
				int[] pxCoordinates = FindClosestPxCoordinates(coordArray[0], coordArray[1]);
				int i = (pxCoordinates[0]-_bottomLeftX)*2;
				int j = img.Height-1-((pxCoordinates[1]-_bottomLeftY)*2);//j index of image goes from top to bottom
										
				Color color = img.GetPixel(i,j); //binary int value						
				lazReader.point.rgb = new [] {
					(ushort) (color.R << 8), 
					(ushort) (color.G << 8),
					(ushort) (color.B << 8), 
					(ushort) 0
				};

				if (IncludeNormals)
				{
					var kNeighbours = kdTree.ApproximateNearest(coordArray, 20, 1000);
					var normal = GetNormal(coordArray, kNeighbours);
								
					var xt = (float) normal[0];//xt in LAS is float
					var yt = (float) normal[1];
					var zt = (float) normal[2];

					var xtBytes = BitConverter.GetBytes(xt);
					var ytBytes = BitConverter.GetBytes(yt);
					var ztBytes = BitConverter.GetBytes(zt);

					var waveformPacket = lazWriter.point.wave_packet;
					waveformPacket[17] = xtBytes[0];
					waveformPacket[18] = xtBytes[1];
					waveformPacket[19] = xtBytes[2];
					waveformPacket[20] = xtBytes[3];
				
					waveformPacket[21] = ytBytes[0];
					waveformPacket[22] = ytBytes[1];
					waveformPacket[23] = ytBytes[2];
					waveformPacket[24] = ytBytes[3];
				
					waveformPacket[25] = ztBytes[0];
					waveformPacket[26] = ztBytes[1];
					waveformPacket[27] = ztBytes[2];
					waveformPacket[28] = ztBytes[3];
				}
				lazWriter.laszip_write_point();
			}
			lazReader.laszip_close_reader();
			lazWriter.laszip_close_writer();
						
			Console.WriteLine("[DONE]");		
		}//end readwrite function

		

	    //simple visualtisation with lasview.exe
	    //param example: "laz13.laz"
	    private static void RunLasview(string fileName)
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

	    //trasnform from LAS 1.2 to 1.3, save new file to folder
	    private static void RunLas2Las()
	    {
		    Console.Write("[{0:hh:mm:ss}] Converting to LAS 1.3 ...", DateTime.Now);
		    var start = new ProcessStartInfo
		    {
			    Arguments = "-i \"" + ResourceDirectoryPath + 
			              "laz12.laz\" -set_point_type 5 -set_version 1.3 -o \""+ ResourceDirectoryPath + "laz13.laz\"",
			    FileName = ResourceDirectoryPath + "las2las",
			    WindowStyle = ProcessWindowStyle.Hidden,
			    CreateNoWindow = false
		    };

		    var process = Process.Start(start);
		    process?.WaitForExit();
		    Console.WriteLine("[DONE]");
	    }

	    //param example: "gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_462_104.laz"
	    private static void Start(string lidarUrl)
	    {
	        DownloadLaz(lidarUrl);
	        SetParameters(lidarUrl);
	        RunLas2Las();
	        ReadWriteLaz();
//	        RunLasview(fileName + ".laz");
	    }
	    
        public static void Main()
        {
	        Console.WriteLine("[{0:hh:mm:ss}] Start program. ", DateTime.Now);        
	        Console.WriteLine("[{0:hh:mm:ss}] Searching for valid ARSO Urls...", DateTime.Now);

	        _addedBlocs = 0;
	        for (var x = SlovenianMapBounds[0]; x <= SlovenianMapBounds[2]; x++)
	        {
		        for (var y = SlovenianMapBounds[1]; y <= SlovenianMapBounds[3]; y++)
		        {
			        var url = GetArsoUrl(x + "_" + y);
			        if (url != null)
			        {
				        Console.WriteLine("[{0:hh:mm:ss}] Found URL: {1}", DateTime.Now, url);
				        Start(url);
				        _addedBlocs++;
				        Console.WriteLine("[{0:hh:mm:ss}] Number of blocs proccesed:  {1}\n", DateTime.Now, _addedBlocs);
			        }
		        }
	        }        
	        Console.WriteLine("[{0:hh:mm:ss}] End program.", DateTime.Now);
        }//end main
	    
	    //nearest neighbour interpolation
	    private static int[] FindClosestPxCoordinates(double x, double y){

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
			    if (IsPointOutOfBounds(currPoint)) continue;
			    var currDistance = (p - currPoint).Length;
			    if (currDistance < minDistance)
			    {
				    closestPoint = currPoint;
				    minDistance = currDistance;
			    }
			    
		    }
		    return new []{(int)closestPoint.X, (int)closestPoint.Y};
	    }
	    
	    //p is out of bounds if x or y coordinate is bigger than width of length of image 
	    private static bool IsPointOutOfBounds(Point p)
	    {
		    double maxX = _bottomLeftX + (OrtoPhotoImgSize-1);
		    double maxY = _bottomLeftY + (OrtoPhotoImgSize-1);

		    return p.X > maxX || p.Y > maxY;
	    }

	    //download and return Image created based on bounds -> _bottomLeftX, _bottomLeftY
	    private static Bitmap GetOrthophotoImg()
	    {
		    double minX = _bottomLeftX;
		    double minY = _bottomLeftY;
		    double maxX = minX + 999.999999999;
		    double maxY = minY + 999.999999999;
		    
			Console.Write("[{0:hh:mm:ss}] Downloading image...", DateTime.Now);
		    var request = WebRequest.Create("http://gis.arso.gov.si/arcgis/rest/services/DOF_2016/MapServer/export" +
		                                       $"?bbox={minX}%2C{minY}%2C{maxX}%2C{maxY}&bboxSR=&layers=&layerDefs=" +
		                                       $"&size={OrtoPhotoImgSize}%2C{OrtoPhotoImgSize}&imageSR=&format=png" +
		                                       "&transparent=false&dpi=&time=&layerTimeOptions=" +
		                                       "&dynamicLayers=&gdbVersion=&mapScale=&f=image");
		    WebResponse response = request.GetResponse();
		    Stream responseStream = response.GetResponseStream();
		    Console.WriteLine("[DONE]");
		    return new Bitmap(responseStream ?? throw new Exception());
	    }

	    //download LAZ file, based on valid URL
	    //param example: "gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_462_104.laz"
	    private static void DownloadLaz(string lidarUrl)
	    {
		    Uri uri = new Uri(lidarUrl);		    
		    Console.Write("[{0:hh:mm:ss}] Downloading Laz from ARSO...", DateTime.Now);
		    WebClient client = new WebClient ();
		    client.DownloadFile(uri, ResourceDirectoryPath + "laz12.laz");
		    Console.WriteLine("[DONE]");
	    }

	    //returns normal of the point, based on the neighbours in KDtree
	    private static double[] GetNormal(double[] point, KDTreeNodeCollection<KDTreeNode> neighbors)
	    {
		    var covarianceMatrix = Matrix.Create(3, 3, 0.0);
		    foreach (var node in neighbors)
		    {
			    var piX = node.Node.Position[0];
			    var piY = node.Node.Position[1];
			    var piZ = node.Node.Position[2];

			    var matrix = Matrix.Create(new[]{new []{piX-point[0]}, new[]{piY-point[1]}, new[] {piZ-point[2]}});
			 	var currC = matrix.DotWithTransposed(matrix);			    
			    covarianceMatrix = currC.Add(covarianceMatrix);
		    }		    
		    var decomposition = new EigenvalueDecomposition(covarianceMatrix,true, true,true);//already sorted - descending order

			var normalX = decomposition.Eigenvectors[0,2];
		    var normalY = decomposition.Eigenvectors[1,2];
		    var normalZ = decomposition.Eigenvectors[2,2];

		    return new []{normalX, normalY,normalZ};
	    }

	    //we use ARSO search bar functionality to find valid URLs, based on brute-forced search terms 
	    // param: example: "470_12"
	    private static string GetArsoUrl(string searchTerm)
	    {
		    try
		    {
			    var client = new RestClient("http://gis.arso.gov.si");
			    var request = new RestRequest("evode/WebServices/NSearchWebService.asmx/GetFilterListItems", Method.POST);
		   
			    request.AddParameter("aplication/json", "{\"configID\":\"lidar_D48GK\",\"culture\":\"sl-SI\",\"groupID\":" +
			                                            "\"grouplidar48GK\"," + "\"parentID\":-1,\"filter\":\""+searchTerm+"\",\"lids\":null,\"sortID\":null}",
				ParameterType.RequestBody);

			    request.AddHeader("Content-Type", "application/json; charset=utf-8");
			    JsonDeserializer deserial = new JsonDeserializer();
		    
			    IRestResponse response = client.Execute(request);
			    var json = deserial.Deserialize<Dictionary<string, Dictionary<string, string>>>(response);
			    if (json["d"]["Count"] == "0")
			    {
				    return null;//search term doesn't exist
			    }
			    var blok = json["d"]["Items"].Split(' ')[2];
			    return "http://gis.arso.gov.si/lidar/gkot/laz/"+ blok + "/D48GK/GK_"+ searchTerm +".laz";

		    }
		    catch (Exception e)
		    {
			    return null;//probably network error
		    }
	    }
    } //end class
}