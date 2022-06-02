

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS


#include "ImGuiApp/ImGuiApp.hpp"
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgViewer/Viewer>
#include <osgEarthDrivers/cache_filesystem/FileSystemCache>
#include <osgEarth/LabelNode>
#include <osgEarth/LogarithmicDepthBuffer>
#include <osgEarth/ModelNode>
#include <osgEarth/Sky>
#include <osgEarth/MBTiles>
#include <osgDB/ReadFile>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ParticleSystemUpdater>
#include <osg/MatrixTransform>
#include <osgEarth/ModelLayer>
#include <osgEarth/GDAL>
#include <osgEarth/WMS>
#include <osgEarth/TMS>
#include <osgEarth/PlaceNode>
#include <osgEarth/GeometryFactory>
#include <osgEarth/Feature>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osgEarth/GeometryRasterizer>
#include <osgEarth/TrackNode>
//#include <osgViewer/CompositeViewer>
//#include <filesystem>
#include <iostream>
#include <osgEarth/AutoClipPlaneHandler>
#include <osgEarthDrivers/sky_simple/SimpleSkyOptions>
#include <osgEarth/Metrics>
//#include <osgFX/Outline>


#define LC "[imgui] "

using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Util;
//namespace fs = std::filesystem;




//////////////////////////////////////////////////////////////////////////////////////



osg::ref_ptr<SpatialReference> srs = SpatialReference::create("WGS84", "EGM96");


int main(int argc, char** argv)
{


	osgEarth::initialize();

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();


	unsigned int scrWidth;
	unsigned int scrHeight;
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->screenNum = 0; // second screen
	traits->x = 0;
	traits->y = 0;
	traits->width = 2560;
	traits->height = 1024;
	traits->windowDecoration = false;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	viewer->getCamera()->setGraphicsContext(gc.get());

	osg::ref_ptr<EarthManipulator> earthManipulator = new EarthManipulator();
	viewer->setCameraManipulator(earthManipulator);

	osg::ref_ptr<osg::Group> group = new osg::Group();


	GeoPoint point(srs, -79, 43, 4000, ALTMODE_ABSOLUTE);

	osg::ref_ptr<LabelNode> label = new LabelNode("Nebi\nSarikaya");


	label->setPosition(point);

	group->addChild(label);

	osg::ref_ptr<osg::Node> globe = osgDB::readNodeFile("bing.earth");


	osg::ref_ptr<MapNode> mapNode = MapNode::get(globe);


	mapNode->getMap()->setCache(mapCache);

	osg::ref_ptr<osgEarth::TMSElevationLayer> elevationLayer = (osgEarth::TMSElevationLayer*)mapNode->getMap()->getLayerAt(0);
	elevationLayer->setName("Elevation Layer");
	elevationLayer->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	elevationLayer->setCacheID("Elevation-Layer");

	osg::ref_ptr<osgEarth::WMSImageLayer> bingLayer = (osgEarth::WMSImageLayer*)mapNode->getMap()->getLayerAt(1);
	bingLayer->setName("Bing Layer");
	bingLayer->setAsyncLoading(true);
	bingLayer->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	bingLayer->setCacheID("Bing-Imagery");

	osg::ref_ptr<osgEarth::WMSImageLayer> arcGISLayer = (osgEarth::WMSImageLayer*)mapNode->getMap()->getLayerAt(2);
	arcGISLayer->setName("ArcGIS Layer");
	arcGISLayer->setAsyncLoading(true);
	arcGISLayer->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	arcGISLayer->setCacheID("ArcGIS-Layer");

        osg::ref_ptr<osgEarth::MBTilesImageLayer> vfrSectionalLayer = new osgEarth::MBTilesImageLayer;
	vfrSectionalLayer->setURL("../../../resources/maps/mbtiles/Detroit SEC 101.mbtiles");
	vfrSectionalLayer->setName("VFR Sectional Layer");
	vfrSectionalLayer->setAsyncLoading(true);
	vfrSectionalLayer->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	vfrSectionalLayer->setCacheID("VFR-Sectional-Layer");

	mapNode->getMap()->addLayer(vfrSectionalLayer);

	point.set(srs, -81.1386835371799, 43.02443500443512, 4000, ALTMODE_ABSOLUTE);

	osg::ref_ptr<osg::Group> root = new osg::Group;


	//////////////////////////////////////////////////////////////////////////////////
	SimpleSky::SimpleSkyOptions _skyOptions;
	_skyOptions.hours() = 19.0f;
	_skyOptions.exposure() = 3.215f;
	_skyOptions.ambient() = 0.5f;
	osg::ref_ptr<SkyNode> sky = SkyNode::create(_skyOptions);
	sky->addChild(root);

	sky->addChild(group);

	group->addChild(mapNode);

	Style labelStyle;
	labelStyle.getOrCreate<TextSymbol>()->size() = 14.0;
	osg::ref_ptr<LabelNode> RWY33 = new LabelNode(GeoPoint(srs, -81.1386835371799, 43.02443500443512, 910 * 0.3048), "RWY33", labelStyle);
	osg::ref_ptr<LabelNode> RWY15 = new LabelNode(GeoPoint(srs, -81.15957085977567, 43.041904904428904, 910 * 0.3048), "RWY15", labelStyle);
	osg::ref_ptr<LabelNode> CYXU = new LabelNode(GeoPoint(srs, -81.14969136937144, 43.03367839385314, 910 * 0.3048), "CYXU", labelStyle);
	group->addChild(RWY33);
	group->addChild(RWY15);
	group->addChild(CYXU);

	/////////////////////////////////////////////////

	// Call this to enable ImGui rendering.
	// If you use the MapNodeHelper, call this first.
	viewer->setRealizeOperation(new GUI::ApplicationGUI::RealizeOperation);

	// Call this to add the GUI. 
	// Passing "true" tells it to install all the built-in osgEarth GUI tools.
	// Put it on the front of the list so events don't filter
	// through to other handlers.

	viewer->getEventHandlers().push_front(new GUI::ApplicationGUI(true));

	LogarithmicDepthBuffer ldbf;
	ldbf.install(viewer->getCamera());


	viewer->setSceneData(sky);


	return viewer->run();

}
#endif
