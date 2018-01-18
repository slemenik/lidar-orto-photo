using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Net;
using System.Runtime.Remoting.Messaging;
using System.Threading;
using System.Web.UI.DataVisualization.Charting;
using Accord.Collections;
using Accord.IO;
using Accord.Math;
using laszip.net;
using RestSharp;
using RestSharp.Deserializers;
using Point = System.Windows.Point;


namespace lidar_orto_photo
{
	internal class Program
    {
	    private static readonly string ResourceDirectoryPath = Directory.GetParent(Directory.GetCurrentDirectory()).Parent?.FullName + "\\resources\\";	   
        
//	    private static readonly string ArsoLidarUrl = "http://gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_459_100.laz";
//	    private static readonly string ArsoLidarUrl = "http://gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_470_97.laz";
//	    private static readonly string ArsoLidarUrl = "http://gis.arso.gov.si/lidar/gkot/laz/b_35/D48GK/GK_462_104.laz";

	    private static readonly object SlovenianMapBounds = new {MinX = 374,MinY = 30,MaxX = 624,MaxY = 194}; //minx,miny,maxx,maxy in thousand

	    private const int OrtoPhotoImgSize = 2000;

	    private static int _bottomLeftX;
	    private static int _bottomLeftY;

	    private static void SetParameters(string fileName)
	    {

		    Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-US");
		    _bottomLeftX = int.Parse(fileName.Split('_')[1]) * 1000;
		    _bottomLeftY = int.Parse(fileName.Split('_')[2]) * 1000;
		    
	    }


	    private static void ReadWriteLaz(string fileName)
		{
			var lazReader = new laszip_dll();
			var lazWriter = new laszip_dll();
			var compressed = true;
			
			var filePath = ResourceDirectoryPath + fileName;
			lazReader.laszip_open_reader(filePath, ref compressed);
			var numberOfPoints = lazReader.header.number_of_point_records;

			var classification = 0;
			var coordArray = new double[3];

			var img = GetOrthophotoImg();
//				var img = new Bitmap(ResourceDirectoryPath + imageName, true);

			lazWriter.header.number_of_point_records = numberOfPoints;
//			Console.WriteLine(numberOfPoints);
			lazWriter.header = lazReader.header;

			var pointsCoordArr = new double[numberOfPoints,3];
			KDTree kdTree = new KDTree(3);
			var newFileName = Path.GetFileNameWithoutExtension(fileName) + "_new.laz";
			lazWriter.laszip_open_writer(ResourceDirectoryPath + newFileName, true);
			
			Console.Write("[{0:hh:mm:ss}] Reading LAZ ...", DateTime.Now);
			for (var pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
			{
				lazReader.laszip_read_point();
				lazReader.laszip_get_coordinates(coordArray);			
				
				pointsCoordArr[pointIndex,0] = coordArray[0];
				pointsCoordArr[pointIndex,1] = coordArray[1];
				pointsCoordArr[pointIndex,2] = coordArray[2];
				
				kdTree.Add(new []{coordArray[0], coordArray[1],coordArray[2]});
				
			}
//			Console.Write(kdTree.Count);
			Console.Write("[DONE] \n[{0:hh:mm:ss}] Writing LAZ...", DateTime.Now);
//			HashSet<int> set = new HashSet<int>();
			lazReader.laszip_seek_point(0L);//read from the beginning
			lazReader.laszip_open_reader(filePath, ref compressed);
			for (int pointIndex = 0; pointIndex < numberOfPoints; pointIndex = pointIndex + 1)
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


				var nearest = kdTree.ApproximateNearest(coordArray, 10, kdTree.Count);
//				if (nearest.Count > 1)
//				{
////					Console.WriteLine(nearest[0].Node.Position[0]);
////					Console.WriteLine(nearest[0].Node.Position[1]);
////					Console.WriteLine(nearest[0].Node.Position[2]);
////					set.Add(nearest.Count);
////					Console.WriteLine(coordArray[0]);
////					Console.WriteLine(coordArray[1]);
////					Console.WriteLine(coordArray[2]);
//					Console.WriteLine(nearest.Count);
//				}
				
				
				var waveformPacket = lazWriter.point.wave_packet;
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
				
//				if (classification == 0)
//				{
////							break;
//				}
//						
				lazWriter.laszip_write_point();
			}
//			Console.WriteLine(set.Count);
			lazReader.laszip_close_reader();
//			lazWriter.laszip_close_writer();
						
			Console.WriteLine("[DONE]");		
		}//end function

		

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

	    private static void RunLas2Las(string fileName)
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
		    process?.WaitForExit();
		    Console.WriteLine("[DONE]");
	    }

	    private static void Start(string lidarUrl)
	    {
//		    var fileName = "GK_459_100";
	        var fileName = DownloadLaz(lidarUrl);
	        SetParameters(fileName);
	        RunLas2Las(fileName + ".laz");
	        ReadWriteLaz(fileName + "_1.laz");
//	        RunLasview(fileName + "_1_new.laz");

	    }
	    
        public static void Main()
        {
	        Console.WriteLine("[{0:hh:mm:ss}] Start program. ", DateTime.Now);

//	        for(int x = SlovenianMapBounds[]; )
//	        covarianceMatrix(null, null);
//	        getArsoUrl("");

	        Console.WriteLine("[{0:hh:mm:ss}] End program.", DateTime.Now);
        }//end main
	    
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
	    
	    private static bool IsPointOutOfBounds(Point p)
	    {
		    double maxX = _bottomLeftX + (OrtoPhotoImgSize-1);
		    double maxY = _bottomLeftY + (OrtoPhotoImgSize-1);

		    return p.X > maxX || p.Y > maxY;
	    }

	    public void NormalEstimation()
	    {
		    var normals = new List<Point3D>();//List<Vector3d> Normals = new List<Vector3d>();Form an empty list of normal vectors
//		    Point3dList PCList = new Point3dList();
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

	    private static string DownloadLaz(string lidarUrl)
	    {
		    Uri uri = new Uri(lidarUrl);
			string filename = Path.GetFileNameWithoutExtension(uri.LocalPath);
		    
		    Console.Write("[{0:hh:mm:ss}] Downloading Laz from ARSO...", DateTime.Now);
		    WebClient client = new WebClient ();
		    client.DownloadFile(uri, ResourceDirectoryPath + filename + ".laz");
		    Console.WriteLine("[DONE]");
		    return filename;
	    }

	    private static void Interpolation()
	    {
//		    public static nearest_nn = None          // # nearest neighbor (NN)
//	    public static double distance_nn = 1000;//  # distance from NN to target
// 
//		public void nearest_neighbor_search(tree, target_point, hr, double distance, nearest=None, depth=0)://nearest=None, depth=0 ko kličemo
//			""" Find the nearest neighbor for the given point (claims O(log(n)) complexity)
//			:param tree         K-D tree
//			:param target_point given point for the NN search
//			:param hr           splitting hyperplane
//			:param distance     minimal distance
//			:param nearest      nearest point
//			:param depth        tree's depth
//			"""
//		 
//			global nearest_nn
//			global distance_nn
//		 
//			if tree is None:
//				return
//		 
//			k = len(target_point)
//		 
//			cur_node = tree.location         # current tree's node
//			left_branch = tree.left_child    # its left branch
//			right_branch = tree.right_child  # its right branch
//		 
//			nearer_kd = further_kd = None
//			nearer_hr = further_hr = None
//			left_hr = right_hr = None
//		 
//			# Select axis based on depth so that axis cycles through all valid values
//			axis = depth % k
//		 
//			# split the hyperplane depending on the axis
//			if axis == 0:
//				left_hr = [hr[0], (cur_node[0], hr[1][1])]
//				right_hr = [(cur_node[0],hr[0][1]), hr[1]]
//		 
//			if axis == 1:
//				left_hr = [(hr[0][0], cur_node[1]), hr[1]]
//				right_hr = [hr[0], (hr[1][0], cur_node[1])]
//		 
//			# check which hyperplane the target point belongs to
//			if target_point[axis] <= cur_node[axis]:
//				nearer_kd = left_branch
//				further_kd = right_branch
//				nearer_hr = left_hr
//				further_hr = right_hr
//		 
//			if target_point[axis] > cur_node[axis]:
//				nearer_kd = right_branch
//				further_kd = left_branch
//				nearer_hr = right_hr
//				further_hr = left_hr
//		 
//			# check whether the current node is closer
//			dist = (cur_node[0] - target_point[0])**2 + (cur_node[1] - target_point[1])**2
//		 
//			if dist < distance:
//				nearest = cur_node
//				distance = dist
//		 
//			# go deeper in the tree
//			nearest_neighbor_search(nearer_kd, target_point, nearer_hr, distance, nearest, depth+1)
//		 
//			# once we reached the leaf node we check whether there are closer points
//			# inside the hypersphere
//			if distance < distance_nn:
//				nearest_nn = nearest
//				distance_nn = distance
//		 
//			# a nearer point (px,py) could only be in further_kd (further_hr) -> explore it
//			px = compute_closest_coordinate(target_point[0], further_hr[0][0], further_hr[1][0])
//			py = compute_closest_coordinate(target_point[1], further_hr[1][1], further_hr[0][1])
//		 
//			# check whether it is closer than the current nearest neighbor => whether a hypersphere crosses the hyperplane
//			dist = (px - target_point[0])**2 + (py - target_point[1])**2
//		 
//			# explore the further kd-tree / hyperplane if necessary
//			if dist < distance_nn:
//				nearest_neighbor_search(further_kd, target_point, further_hr, distance, nearest, depth+1)
		    
		    
	    }

	    private static void covarianceMatrix(double[] point, KDTreeNodeCollection<KDTreeNode> neighbors)
	    {
		    Matrix3x3 C = new Matrix3x3();
		    Console.WriteLine(C.V00);
			Vector3 p = new Vector3((float) point[0], (float) point[1], (float) point[2]);
//		    var iterator = neighbors;

		    foreach (var node in neighbors)
		    {
			    Vector3 pi = new Vector3((float) node.Node.Position[0], (float) node.Node.Position[1],(float) node.Node.Position[2]);
			    Vector3 matrix = pi - p;
			    

		    }
		    
		    
	    }

	    private static string getArsoUrl(string searchTerm)
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
			    return null;
		    }
		    var blok = json["d"]["Items"].Split(' ')[2];
		    return "http://gis.arso.gov.si/lidar/gkot/laz/"+ blok + "/D48GK/GK_"+ searchTerm +".laz";

	    }

    } //end class
}