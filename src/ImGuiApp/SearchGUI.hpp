/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#ifndef OSGEARTH_IMGUI_SEARCH_GUI
#define OSGEARTH_IMGUI_SEARCH_GUI

#include "ImGuiImp.hpp"
#include <osgEarth/Utils>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Geocoder>
#include <osgEarth/ViewFitter>
#include <osgEarth/MapNode>

namespace osgEarth {
    namespace GUI
    {
        using namespace osgEarth;
        using namespace osgEarth::Util;

        class SearchGUI : public BaseGUI
        {
        public:
            SearchGUI() :
                BaseGUI("Search"),
                _search(""),
                _results(Status::Code::ServiceUnavailable, 0)
            {
            }

            void draw(osg::RenderInfo& ri) override
            {
                if (!isVisible())
                    return;

                if (!_arena)
                {
                    _options = new osgDB::Options();
                    _arena = std::make_shared<JobArena>("oe.geocoder", 1U);
                    ObjectStorage::set(_options.get(), _arena);
                }

                if (!_mapSRS.valid()) {
                    MapNode* mapNode = findNode<MapNode>(ri);
                    if (mapNode)
                        _mapSRS = mapNode->getMapSRS();
                    else {
                        setVisible(false);
                        return;
                    }
                }

                ImGui::Begin(name(), visible());

                ImGui::Text("Enter place name:");
                if (ImGui::InputText("", _search, 128,
                    ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll))
                {
                    _features.clear();
                    _results = geocoder.search(_search, _options.get());
                }

                if (_results.isReady())
                {
                    if (_results.getStatus().isOK())
                    {
                        _results.getFeatures()->fill(_features);
                    }
                    _results = Geocoder::Results(Status::Code::ServiceUnavailable, 0);
                }

                else if (_results.isWorking())
                {
                    ImGui::Text("Searching...");
                }

                for (FeatureList::iterator itr = _features.begin(); itr != _features.end(); ++itr)
                {
                    bool selected = false;
                    Feature* feature = itr->get();
                    if (feature->getGeometry())
                    {
                        ImGui::Separator();
                        std::string displayName = itr->get()->getString("display_name");
                        ImGui::Selectable(displayName.c_str(), &selected);
                        if (selected)
                        {
                            GeoExtent extent(feature->getSRS(), feature->getGeometry()->getBounds());
                            if (extent.area() == 0.0)
                                extent.expand(Distance(10, Units::KILOMETERS), Distance(10, Units::KILOMETERS));
                            ViewFitter fitter(_mapSRS.get(), ri.getCurrentCamera());
                            Viewpoint vp;
                            if (fitter.createViewpoint(extent, vp))
                            {
                                auto manip = dynamic_cast<EarthManipulator*>(view(ri)->getCameraManipulator());
                                manip->setViewpoint(vp, 3.0f);
                            }
                            break;
                        }
                    }
                }

                ImGui::End();
            }

            char _search[128];
            Geocoder::Results _results;
            osg::ref_ptr<osgDB::Options> _options;
            std::shared_ptr<JobArena> _arena;
            FeatureList _features;
            Geocoder geocoder;
            osg::ref_ptr<const SpatialReference> _mapSRS;
        };
    }
}

#endif // OSGEARTH_IMGUI_SEARCH_GUI
