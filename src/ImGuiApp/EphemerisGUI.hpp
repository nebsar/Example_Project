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
#ifndef OSGEARTH_IMGUI_EPHEMERIS_GUI
#define OSGEARTH_IMGUI_EPHEMERIS_GUI

#include "ImGuiImp.hpp"
#include <osgEarth/Ephemeris>
#include <osgEarth/Sky>
#include <osgEarth/Shadowing>
#include <osgEarth/WindLayer>

namespace
{
    const char* render_wind = R"(
#version 450
#pragma vp_function oe_ui_render_wind_vert, vertex_view
out vec3 viewpos3_wind;
void oe_ui_render_wind_vert(inout vec4 vertex) {
    viewpos3_wind = vertex.xyz;
}
[break]
#version 450
#pragma vp_function oeui_render_wind_texture, fragment_output
#pragma import_defines(OE_WIND_TEX)
#pragma import_defines(OE_WIND_TEX_MATRIX)
in vec3 viewpos3_wind;
out vec4 frag_out;
#ifdef OE_WIND_TEX
uniform sampler3D OE_WIND_TEX;
uniform mat4 OE_WIND_TEX_MATRIX;
#endif
void oeui_render_wind_texture(inout vec4 color) {
    frag_out = color;
#ifdef OE_WIND_TEX
    vec4 texel = textureProj(OE_WIND_TEX, (OE_WIND_TEX_MATRIX*vec4(viewpos3_wind,1)));
    vec3 wind = normalize(texel.xyz);
    frag_out = vec4(wind, 1); //mix(color, wind, 0.8f);
#endif
}
)";
}

namespace osgEarth
{
    namespace GUI
    {

        using namespace osgEarth;
        using namespace osgEarth::Util;

        class EphemerisGUI : public BaseGUI
        {
        private:
            osg::observer_ptr<SkyNode> _skyNode;
            osg::observer_ptr<ShadowCaster> _shadowCaster;
            osg::observer_ptr<WindLayer> _windLayer;
            bool _showDetails;
            float _hour;
            int _day, _month, _year;
            float _exposure;
            float _contrast;
            float _gamma;
            float _ambient;
            bool _first;
            bool _shadows;
            float _haze_cutoff, _haze_strength;
            float _shadow_darkness, _shadow_blur;
            float _wind_power;

        public:
            EphemerisGUI() : BaseGUI("Environment"),
                _ambient(0.033f),
                _exposure(3.5f),
                _contrast(1.0f),
                _haze_cutoff(0.0f),
                _haze_strength(16.0f),
                _showDetails(false),
                _shadows(false),
                _shadow_darkness(0.5f),
                _shadow_blur(0.001f),
                _wind_power(1.0f),
                _first(true)
            {
                DateTime now;
                _hour = now.hours(), _day = now.day(), _month = now.month(), _year = now.year();
            }

            void load(const Config& conf) override
            {
                conf.get("ShowDetails", _showDetails);
                conf.get("Hour", _hour);
                conf.get("Day", _day);
                conf.get("Month", _month);
                conf.get("Year", _year);
                conf.get("Exposure", _exposure);
                conf.get("Contrast", _contrast);
                conf.get("Ambient", _ambient);
                conf.get("HazeCutoff", _haze_cutoff);
                conf.get("HazeStrength", _haze_strength);
                conf.get("WindPower", _wind_power);
            }
            void save(Config& conf) override
            {
                conf.set("ShowDetails", _showDetails);
                conf.set("Hour", _hour);
                conf.set("Day", _day);
                conf.set("Month", _month);
                conf.set("Year", _year);
                conf.set("Exposure", _exposure);
                conf.set("Contrast", _contrast);
                conf.set("Ambient", _ambient);
                conf.set("HazeCutoff", _haze_cutoff);
                conf.set("HazeStrength", _haze_strength);
                conf.set("WindPower", _wind_power);
            }

            void draw(osg::RenderInfo& ri) override
            {
                if (!isVisible() || !findNodeOrHide(_skyNode, ri))
                    return;

                if (_first)
                {
                    findNode(_shadowCaster, ri);
                    if (_shadowCaster.valid())
                        _shadows = _shadowCaster->getEnabled();

                    findLayer(_windLayer, ri);

                    _skyNode->setDateTime(DateTime(_year, _month, _day, _hour));

                    // so we can visualize tiume-series layers.
                    _skyNode->setSimulationTimeTracksDateTime(true);
                }

                ImGui::Begin(name(), visible());
                {   
                    bool lighting = _skyNode->getLighting();
                    ImGui::Checkbox("Lighting", &lighting);
                    _skyNode->setLighting(lighting);

                    if (_shadowCaster.valid()) {
                        ImGui::SameLine();
                        ImGui::Checkbox("Shadows", &_shadows);
                        _shadowCaster->setEnabled(_shadows);
                    }

                    ImGui::SameLine();
                    if (ImGui::Checkbox("Details", &_showDetails)) dirtySettings();

                    ImGui::Separator();

                    if (_showDetails)
                        ImGui::Text("Date & Time:");

                    if (ImGui::SliderFloat("Hour", &_hour, 0.0f, 24.0f))
                        dirtySettings();

                    if (_showDetails)
                    {
                        if (ImGui::SliderInt("Day", &_day, 1, 31)) dirtySettings();
                        if (ImGui::SliderInt("Month", &_month, 1, 12)) dirtySettings();
                        if (ImGui::SliderInt("Year", &_year, 1970, 2061)) dirtySettings();
                    }

                    DateTime mark(_year, _month, _day, _hour);
                    _skyNode->setDateTime(mark);

                    if (lighting)
                    {
                        if (ImGui::SliderFloat("Exposure", &_exposure, 1.0f, 10.0f)) dirtySettings();
                        _skyNode->getOrCreateStateSet()->getOrCreateUniform("oe_sky_exposure", osg::Uniform::FLOAT)->set(_exposure);

                        if (ImGui::SliderFloat("Ambient", &_ambient, 0.0f, 1.0f)) dirtySettings();
                        _skyNode->getSunLight()->setAmbient(osg::Vec4(_ambient, _ambient, _ambient, 1.0f));
                    }
                    else
                    {
                        _shadows = false;
                    }

                    if (_windLayer.valid())
                    {
                        if (ImGui::SliderFloat("Wind", &_wind_power, 1.0f, 10.0f) || _first)
                        {
                            stateset(ri)->addUniform(new osg::Uniform("oe_wind_power", _wind_power),
                                osg::StateAttribute::OVERRIDE | 0x01);
                        }

                        static bool show_wind = false;
                        if (ImGui::Checkbox("Wind debug view", &show_wind))
                        {
                            if (show_wind)
                                ShaderLoader::load(VirtualProgram::getOrCreate(stateset(ri)), render_wind);
                            else
                                ShaderLoader::unload(VirtualProgram::getOrCreate(stateset(ri)), render_wind);
                        }
                    }

                    if (_showDetails)
                    {
                        ImGui::Separator();

                        if (_shadows)
                        {
                            if (ImGui::SliderFloat("Shadow darkness", &_shadow_darkness, 0.0f, 1.0f))
                                stateset(ri)->addUniform(new osg::Uniform("oe_shadow_color", _shadow_darkness), 0x7);
                            if (ImGui::SliderFloat("Shadow blur", &_shadow_blur, 0.0f, 0.002f))
                                stateset(ri)->addUniform(new osg::Uniform("oe_shadow_blur", _shadow_blur), 0x07);
                        }

                        ImGui::Separator();

                        if (ImGui::SliderFloat("Haze cutoff", &_haze_cutoff, 0.0f, 0.2f)) dirtySettings();
                        _skyNode->getOrCreateStateSet()->getOrCreateUniform("atmos_haze_cutoff", osg::Uniform::FLOAT)->set(_haze_cutoff);

                        if (ImGui::SliderFloat("Haze strength", &_haze_strength, 0.0f, 24.0f)) dirtySettings();
                        _skyNode->getOrCreateStateSet()->getOrCreateUniform("atmos_haze_strength", osg::Uniform::FLOAT)->set(_haze_strength);

                        bool atmos_visible = _skyNode->getAtmosphereVisible();
                        ImGui::Checkbox("Atmosphere", &atmos_visible);
                        _skyNode->setAtmosphereVisible(atmos_visible);

                        bool sun_visible = _skyNode->getSunVisible();
                        ImGui::Checkbox("Sun", &sun_visible);
                        _skyNode->setSunVisible(sun_visible);

                        ImGui::SameLine();
                        bool moon_visible = _skyNode->getMoonVisible();
                        ImGui::Checkbox("Moon", &moon_visible);
                        _skyNode->setMoonVisible(moon_visible);

                        ImGui::SameLine();
                        bool stars_visible = _skyNode->getStarsVisible();
                        ImGui::Checkbox("Stars", &stars_visible);
                        _skyNode->setStarsVisible(stars_visible);

                        ImGui::Separator();

                        DateTime dt = _skyNode->getDateTime();

                        CelestialBody sun = _skyNode->getEphemeris()->getSunPosition(dt);
                        ImGui::Text("Sun: RA (%.2f) Decl (%.2f)",
                            sun.rightAscension.as(Units::DEGREES),
                            sun.declination.as(Units::DEGREES));

                        CelestialBody moon = _skyNode->getEphemeris()->getMoonPosition(dt);
                        ImGui::Text("Moon: RA (%.2f) Decl (%.2f)",
                            moon.rightAscension.as(Units::DEGREES),
                            moon.declination.as(Units::DEGREES));
                    }
                }
                ImGui::End();

                _first = false;
            }
        };
    }
}

#endif // OSGEARTH_IMGUI_EPHEMERIS_GUI
