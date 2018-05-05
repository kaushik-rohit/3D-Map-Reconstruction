#include <rtabmap/core/OdometryF2M.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <QApplication>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "MapBuilder.h"

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-rgbd_mapping driver\n"
			"  driver       Driver number to use: 0=OpenNI-PCL, 1=OpenNI2, 2=Freenect, 3=OpenNI-CV, 4=OpenNI-CV-ASUS, 5=Freenect2, 6=ZED SDK, 7=RealSense\n\n");
	exit(1);
}

using namespace rtabmap;
int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	int driver = 0;
	if(argc < 2)
	{
		showUsage();
	}
	else
	{
		driver = atoi(argv[argc-1]);
		if(driver < 0 || driver > 7)
		{
			UERROR("driver should be between 0 and 7.");
			showUsage();
		}
	}

	// Here is the pipeline that we will use:
	// CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

	// Create the OpenNI camera, it will send a CameraEvent at the rate specified.
	// Set transform to camera so z is up, y is left and x going forward
	Camera * camera = 0;
	Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
	if(driver == 1)
	{
		if(!CameraOpenNI2::available())
		{
			UERROR("Not built with OpenNI2 support...");
			exit(-1);
		}
		camera = new CameraOpenNI2("", CameraOpenNI2::kTypeColorDepth, 0, opticalRotation);
	}
	else if(driver == 2)
	{
		if(!CameraFreenect::available())
		{
			UERROR("Not built with Freenect support...");
			exit(-1);
		}
		camera = new CameraFreenect(0, CameraFreenect::kTypeColorDepth, 0, opticalRotation);
	}
	else if(driver == 3)
	{
		if(!CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new CameraOpenNICV(false, 0, opticalRotation);
	}
	else if(driver == 4)
	{
		if(!CameraOpenNICV::available())
		{
			UERROR("Not built with OpenNI from OpenCV support...");
			exit(-1);
		}
		camera = new CameraOpenNICV(true, 0, opticalRotation);
	}
	else if (driver == 5)
	{
		if (!CameraFreenect2::available())
		{
			UERROR("Not built with Freenect2 support...");
			exit(-1);
		}
		camera = new CameraFreenect2(0, CameraFreenect2::kTypeColor2DepthSD, 0, opticalRotation);
	}
	else if (driver == 6)
	{
		if (!CameraStereoZed::available())
		{
			UERROR("Not built with ZED SDK support...");
			exit(-1);
		}
		camera = new CameraStereoZed(0, 2, 1, 1, 100, false, 0, opticalRotation);
	}
	else if (driver == 7)
	{
		if (!CameraRealSense::available())
		{
			UERROR("Not built with RealSense support...");
			exit(-1);
		}
		camera = new CameraRealSense(0, 0, 0, 0, opticalRotation);
	}
	else
	{
		camera = new rtabmap::CameraOpenni("", 0, opticalRotation);
	}

	if(!camera->init())
	{
		UERROR("Camera init failed!");
	}

	CameraThread cameraThread(camera);


	// GUI stuff, there the handler will receive RtabmapEvent and construct the map
	// We give it the camera so the GUI can pause/resume the camera
	QApplication app(argc, argv);
	MapBuilder mapBuilder(&cameraThread);

	// Create an odometry thread to process camera events, it will send OdometryEvent.
	OdometryThread odomThread(new OdometryF2M());


	// Create RTAB-Map to process OdometryEvent
	Rtabmap * rtabmap = new Rtabmap();
	rtabmap->init();
	RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

	// Setup handlers
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	mapBuilder.registerToEventsManager();

	// The RTAB-Map is subscribed by default to CameraEvent, but we want
	// RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
	// We can do that by creating a "pipe" between the camera and odometry, then
	// only the odometry will receive CameraEvent from that camera. RTAB-Map is
	// also subscribed to OdometryEvent by default, so no need to create a pipe between
	// odometry and RTAB-Map.
	UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

	// Let's start the threads
	rtabmapThread.start();
	odomThread.start();
	cameraThread.start();

	mapBuilder.show();
	app.exec(); // main loop

	// remove handlers
	mapBuilder.unregisterFromEventsManager();
	rtabmapThread.unregisterFromEventsManager();
	odomThread.unregisterFromEventsManager();

	// Kill all threads
	cameraThread.kill();
	odomThread.join(true);
	rtabmapThread.join(true);

	// Save 3D map
	printf("Saving rtabmap_cloud.pcd...\n");
	std::map<int, Signature> nodes;
	std::map<int, Transform> optimizedPoses;
	std::multimap<int, Link> links;
	rtabmap->get3DMap(nodes, optimizedPoses, links, true, true);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(std::map<int, Transform>::iterator iter=optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter)
	{
		Signature node = nodes.find(iter->first)->second;

		// uncompress data
		node.sensorData().uncompressData();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.01f);  // Voxel grid filtering
		*cloud += *util3d::transformPointCloud(tmp, iter->second); // transform the point cloud to its pose
	}
	if(cloud->size())
	{
		//printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
		//cloud = util3d::voxelize(cloud, 0.01f);

		printf("Saving rtabmap_cloud.pcd... done! (%d points)\n", (int)cloud->size());
		pcl::io::savePCDFile("3d-334_4.pcd", *cloud);
		//pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY format
	}
	else
	{
		printf("Saving rtabmap_cloud.pcd... failed! The cloud is empty.\n");
	}

	// Save trajectory
	printf("Saving rtabmap_trajectory.txt ...\n");
	if(optimizedPoses.size() && graph::exportPoses("rtabmap_trajectory.txt", 0, optimizedPoses, links))
	{
		printf("Saving rtabmap_trajectory.txt... done!\n");
	}
	else
	{
		printf("Saving rtabmap_trajectory.txt... failed!\n");
	}

	return 0;
}

