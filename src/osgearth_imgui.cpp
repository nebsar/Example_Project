

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



class UpdateAircraftCallback : public osg::NodeCallback

{

protected:
	double* _heading;
public:
	UpdateAircraftCallback(double* heading) : _heading(heading) {}

	void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osgEarth::TrackNode* tracknode = static_cast<osgEarth::TrackNode*>(node);
		Style pin;

		pin.getOrCreate<IconSymbol>()->url()->setLiteral("F-35A.png");
		IconSymbol* is = pin.getSymbol<IconSymbol>();
		is->heading() = NumericExpression(*_heading);
		pin.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_CENTER_CENTER; //add this line
		tracknode->setStyle(pin);
		//traverse(node, nv);
	}
};
//////////////////////////////////////////////////////////////////////////////////////

void rotateIcon(PlaceNode* pNode) {
	double heading = 0.0;
	while (true) {
		Style pin;

		pin.getOrCreate<IconSymbol>()->url()->setLiteral("F-35A.png");
		IconSymbol* is = pin.getSymbol<IconSymbol>();
		is->heading() = NumericExpression(heading);
		pNode->setStyle(pin);
		//pNode->dirty();
		heading -= 1;
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

}

void increaseHeading(double* heading) {
	while (true)
	{
		*heading = *heading + 0.0000001;

	}

}

osg::ref_ptr<SpatialReference> srs = SpatialReference::create("WGS84", "EGM96");




int main(int argc, char** argv)
{


	osgEarth::initialize();

	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();


	//too get the secons screen position ///////////////////////////////


	unsigned int scrWidth;
	unsigned int scrHeight;
	osg::GraphicsContext::WindowingSystemInterface* wsi =
		osg::GraphicsContext::getWindowingSystemInterface();
	if (wsi->getNumScreens() > 0) {
		wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), scrWidth, scrHeight);
	}
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->screenNum = 0; // second screen
	traits->x = 0;
	traits->y = 0;
	traits->width = 2560;
	traits->height = 1024;
	traits->depth = 24;
	traits->stencil = 1;
	traits->alpha = 8;
	traits->samples = 4;
	traits->glContextVersion = "3.0";
	//traits->sampleBuffers = 2;
	traits->windowDecoration = false;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->setInheritedWindowPixelFormat = true;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	osg::ref_ptr<osg::Camera> mainCamera = new osg::Camera;
	viewer->getCamera()->setGraphicsContext(gc.get());
	viewer->getCamera()->setViewport(new osg::Viewport(0, 0, 2560, 1024));


	////////////////////////////////////////////////////////////////////////////////


	osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);
	viewer->getCamera()->setClearMask(
		GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT
	);
	viewer->getCamera()->setClearStencil(0);

	// Use SingleThreaded mode with imgui.
	//viewer->setThreadingModel(viewer->SingleThreaded);

	osg::ref_ptr<EarthManipulator> earthManipulator = new EarthManipulator();
	earthManipulator->getSettings()->setMinMaxPitch(-360, 360);
	earthManipulator->getSettings()->bindScroll(EarthManipulator::ACTION_ZOOM_IN, osgGA::GUIEventAdapter::SCROLL_UP);
	earthManipulator->getSettings()->bindScroll(EarthManipulator::ACTION_ZOOM_OUT, osgGA::GUIEventAdapter::SCROLL_DOWN);
	earthManipulator->getSettings()->bindMouse(EarthManipulator::ACTION_ZOOM, osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
	//earthManipulator->getSettings()->bindMouse(EarthManipulator::ACTION_ZOOM, osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON);
	earthManipulator->getSettings()->bindMouse(EarthManipulator::ACTION_ROTATE, osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON);
	viewer->setCameraManipulator(earthManipulator);


	// 
	//ldbf->setUseFragDepth(true);
	viewer->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);

	//this line is very important for getting close to ground.
	viewer->getCamera()->setNearFarRatio(0.0000001f);


	osg::ref_ptr<osg::Group> group = new osg::Group();


	GeoPoint point(srs, -79, 43, 4000, ALTMODE_ABSOLUTE);

	osg::ref_ptr<LabelNode> label = new LabelNode("Nebi\nSarikaya");


	label->setPosition(point);

	group->addChild(label);

	osgEarth::Drivers::FileSystemCacheOptions osgEarthCacheOptions;
	osgEarthCacheOptions.rootPath() = "../../../../osgEarthData";
	osgEarth::Registry::instance()->setDefaultCache(osgEarth::CacheFactory::create(osgEarthCacheOptions));
	osgEarth::Registry::instance()->setDefaultCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);

	osg::ref_ptr<Cache> mapCache = CacheFactory::create(osgEarthCacheOptions);

	osg::ref_ptr<osg::Node> globe = osgDB::readNodeFile("../../../resources/maps/earth_file/bing.earth");


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


	//for smoothing the 3d model
	osg::DisplaySettings::instance()->setNumMultiSamples(4);





	osg::ref_ptr<osg::Group> root = new osg::Group;




	//////////////////////////////////////////////////////////////////////////////////
	SimpleSky::SimpleSkyOptions _skyOptions;
	_skyOptions.hours() = 19.0f;
	_skyOptions.exposure() = 3.215f;
	_skyOptions.ambient() = 0.5f;
	osg::ref_ptr<SkyNode> sky = SkyNode::create(_skyOptions);

	//for sky shading effects
	//sky->addChild(outline);
	sky->addChild(root);

	//this is important for seeing the black background
	sky->attach(viewer);


	sky->addChild(group);


	//osg::ref_ptr<GDALElevationLayer> elevationLayer = new GDALElevationLayer;
	//elevationLayer->setURL("elevData.vrt");
	//elevationLayer->setName("Elevation Layer");
	//elevationLayer->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	//mapNode->getMap()->addLayer(elevationLayer);


	/*WMSImageLayer* wms = new WMSImageLayer;
	wms->setURL("http://worldwind27.arc.nasa.gov/wms/virtualearth");
	wms->setLayers("ve-h");
	wms->setName("Bing Image Layer");
	wms->setSecondsPerFrame(0.0);
	wms->setFormat("png");
	wms->setSRS("EPSG:3857");
	wms->setStyle("default");
	wms->setTransparent(false);
	wms->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	mapNode->getMap()->addLayer(wms);*/

	/*osg::ref_ptr<MBTilesImageLayer> layerMbTiles = new MBTilesImageLayer;
	layerMbTiles->setName("New_York VFR Sectional");
	layerMbTiles->setURL("New_York_SEC_98_cc4.mbtiles");
	layerMbTiles->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	mapNode->getMap()->addLayer(layerMbTiles);

	layerMbTiles = new MBTilesImageLayer;

	layerMbTiles->setName("Montreal VFR Sectional");
	layerMbTiles->setURL("Montreal_SEC_99_c26.mbtiles");
	layerMbTiles->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	mapNode->getMap()->addLayer(layerMbTiles);


	layerMbTiles = new MBTilesImageLayer;
	layerMbTiles->setURL("Detroit_SEC_97_c25.mbtiles");
	layerMbTiles->setName("Detroit VFR Sectional");
	layerMbTiles->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);
	mapNode->getMap()->addLayer(layerMbTiles);


	osg::ref_ptr<GDALImageLayer> gdal1 = new GDALImageLayer;
	gdal1->setURL("C:/Users/Nebi.Sarikaya/Downloads/OSGEARTH/IMGUI_OSGEARTH_BUILD/rpf/RPF/A.TOC");
	mapNode->getMap()->addLayer(gdal1);
	gdal1->setName("ATOC");

	osg::ref_ptr<GDALImageLayer> gdal2 = new GDALImageLayer;
	gdal2->setURL("C:/Users/Nebi.Sarikaya/Downloads/OSGEARTH/denememb/onc.jpg");
	mapNode->getMap()->addLayer(gdal2);
	gdal2->setName("ONC");*/

	group->addChild(mapNode);


	/////////////// ADDING PRIZMA /////////////////////////
	float distance = 10; // 60km from point A to BCDE
	float width = 10;       // 21.8km in horizontal plane from A to BC and to DE
	float height = 10;    // 9.77km in vertical plane from A to EC and to DB

	//---------------- Building pyramid in local space -------------
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	/*vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));              // A
	vertices->push_back(osg::Vec3(width, distance, -height));       // B
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));              // A
	vertices->push_back(osg::Vec3(width, distance, height));        // C
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));              // A
	vertices->push_back(osg::Vec3(-width, distance, -height));      // D
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));              // A*/
	vertices->push_back(osg::Vec3(-width, distance, height));      // E
	vertices->push_back(osg::Vec3(-width, distance, -height));     // D
	vertices->push_back(osg::Vec3(width, distance, -height));      // B
	vertices->push_back(osg::Vec3(width, distance, height));       // C
	vertices->push_back(osg::Vec3(-width, distance, height));      // E

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

	geom->setVertexArray(vertices.get());

	//geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, vertices->size()));
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, vertices->size()));

	//We do not have to use Geode...
	//osg::ref_ptr<osg::Geode> geode = new osg::Geode;

	//geode->addDrawable(geom.get());

	//--------------- Adding PositionAttitudeTransform to rotate pyramid ---------
	osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform;
	pat->addChild(geom.get());
	double roll = 0.0f;
	double pitch = 0.0f;
	double heading = 321.0f;
	osg::Quat ori =
		osg::Quat(osg::DegreesToRadians(roll), osg::Vec3(0, 1, 0)) *
		osg::Quat(osg::DegreesToRadians(pitch), osg::Vec3(1, 0, 0)) *
		osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0, 0, -1));
	pat->setAttitude(ori);

	//------------- GeoTransform to set GeoPosition of point A ---------------
	osg::ref_ptr<osgEarth::GeoTransform> geo = new osgEarth::GeoTransform;
	geo->setTerrain(mapNode->getTerrain());
	geo->setPosition(GeoPoint(srs, -81.1386835371799, 43.02443500443512, 400, ALTMODE_ABSOLUTE));
	geo->addChild(pat.get());

	//------------- Add GeoTransform to root group -------------
	group->addChild(geo.get());




	/////////////////////////////////////////////////////////////////////////////////////////


	//////////////////////// ICON SYMBOL ///////////////////////////////////////


	/*Style pin;

	pin.getOrCreate<IconSymbol>()->url()->setLiteral("F-35A.png");
	IconSymbol* is = pin.getSymbol<IconSymbol>();
	is->heading() = NumericExpression(30);
	pin.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_CENTER_CENTER; //add this line
	osgEarth::PlaceNode* pNode = new PlaceNode(point, "1-1", pin);





	///////////////////////////////TRACK NODE ///////////////////////////////////////////////

	/*TrackNodeFieldSchema fieldSchema;

	osg::Image* nodeImage = osgDB::readImageFile("F-35A.png");
	TrackNode* trackNode = new TrackNode(point, nodeImage, fieldSchema);

	 group->addChild(trackNode);*/



	Style labelStyle;
	labelStyle.getOrCreate<TextSymbol>()->size() = 14.0;
	labelStyle.getOrCreate<TextSymbol>()->halo()->color() = osg::Vec4(0, 0, 0, 1);
	labelStyle.getOrCreate<TextSymbol>()->fill()->color() = osg::Vec4(0, 1, 0, 1);
	labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
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
