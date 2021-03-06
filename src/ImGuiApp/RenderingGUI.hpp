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
#ifndef OSGEARTH_IMGUI_RENDERING_GUI
#define OSGEARTH_IMGUI_RENDERING_GUI

#include "ImGuiImp.hpp"
#include <osgEarth/Threading>
#include <osgEarth/Memory>
#include <osgEarth/GLUtils>
#include <osgEarth/ShaderLoader>
#include <chrono>
#include <list>

namespace {
    const char* render_view_normals = R"(
#version 450
#pragma vp_function oeui_render_view_normals, fragment_lighting, last
in vec3 vp_Normal;
void oeui_render_view_normals(inout vec4 color) {
    color = vec4((vp_Normal+1.0)*0.5, 1);
}
)";

    const char* render_model_normals = R"(
#version 450
#pragma vp_function oeui_render_model_normals_vs, vertex_model, last
out vec3 vp_Normal;
out vec3 oeui_model_normal;
void oeui_render_model_normals_vs(inout vec4 vertex) {
    oeui_model_normal = vp_Normal;
}
[break]
#version 450
#pragma vp_function oeui_render_view_normals_fs, fragment_lighting, last
in vec3 oeui_model_normal;
void oeui_render_view_normals_fs(inout vec4 color) {
    color = vec4( (normalize(oeui_model_normal)+1.0)*0.5, 1);
}
)";

    const char* render_fb_normals = R"(
#version 450
#pragma vp_function oeui_render_fb_normals, fragment_lighting, last
in vec3 vp_Normal;
void oeui_render_fb_normals(inout vec4 color) {
    float a = step(0.5, color.a);
    float nz = normalize(vp_Normal).z;
    color.rgb = mix(vec3(0,0,0), vec3(1,1,1), (nz+1.0)*0.5);
    color = vec4(color.rgb, a);
}
)";

    const char* render_winding = R"(
#version 450
#extension GL_NV_fragment_shader_barycentric : enable
#pragma vp_function oeui_render_winding_fs, fragment_lighting, last
void oeui_render_winding_fs(inout vec4 color) {
    color.rgb = gl_FrontFacing ? vec3(0,0.75,0) : vec3(1,0,0);
    float b = min(gl_BaryCoordNV.x, min(gl_BaryCoordNV.y, gl_BaryCoordNV.z))*32.0;
    color = vec4(mix(vec3(1), color.rgb, clamp(b,0,1)), 1.0);
}
)";

    const char* render_outlines = R"(
#version 450
#extension GL_NV_fragment_shader_barycentric : enable
#pragma vp_function oeui_render_outlines, fragment_lighting, last
void oeui_render_outlines(inout vec4 color) {
    float b = min(gl_BaryCoordNV.x, min(gl_BaryCoordNV.y, gl_BaryCoordNV.z))*32.0;
    color = vec4(mix(vec3(1), color.rgb, clamp(b,0,1)), color.a);
}
)";

}

namespace osgEarth
{
    namespace GUI
    {
        using namespace osgEarth;
        using namespace osgEarth::Threading;

        class RenderingGUI : public BaseGUI
        {
        private:
            float _sse;
            osg::observer_ptr<MapNode> _mapNode;
            using time_point = std::chrono::time_point<std::chrono::steady_clock>;
            time_point _lastFrame;
            std::queue<int> _times;
            int _time_accum;
            int _frameCounter;
            int _fps;
            std::string _renderMode;
            bool _renderViewNormals;
            bool _renderModelNormals;
            bool _renderWinding;
            bool _renderOutlines;

        public:
            RenderingGUI() : BaseGUI("Rendering"),
                _sse(0.0f),
                _frameCounter(0), _time_accum(0),
                _renderViewNormals(false), _renderModelNormals(false),
                _renderWinding(false), _renderOutlines(false) { }

            void load(const Config& conf) override
            {
                conf.get("SSE", _sse);
            }

            void save(Config& conf) override
            {
                conf.set("SSE", _sse);
            }                

            void setRenderMode(const std::string& mode, osg::RenderInfo& ri)
            {
                auto* vp = VirtualProgram::getOrCreate(stateset(ri));
                if (!_renderMode.empty())
                    ShaderLoader::unload(vp, _renderMode);
                _renderMode = mode;
                if (!_renderMode.empty())
                    ShaderLoader::load(vp, _renderMode);
            }

            void draw(osg::RenderInfo& ri) override
            {
                if (!isVisible())
                    return;

                if (!findNodeOrHide(_mapNode, ri))
                    return;

                ImGui::Begin(name(), visible());
                {
                    time_point now = std::chrono::steady_clock::now();
                    int us = std::chrono::duration_cast<std::chrono::microseconds>(now - _lastFrame).count();
                    _lastFrame = now;
                    constexpr int interval = 60;
                    _times.push(us);
                    _time_accum += us;
                    if (_times.size() > interval) {
                        _time_accum -= _times.front();
                        _times.pop();
                    }
                    if (_frameCounter++ % interval == 0)
                        _fps = 1000000 / (_time_accum / interval);

                    if (ImGui::BeginTable("render", 2))
                    {
                        ImGui::TableNextColumn();
                        ImGui::Text("FPS: %d", _fps);

                        const osgViewer::ViewerBase::ThreadingModel models[] = {
                            osgViewer::ViewerBase::SingleThreaded,
                            osgViewer::ViewerBase::DrawThreadPerContext,
                            osgViewer::ViewerBase::CullDrawThreadPerContext,
                            osgViewer::ViewerBase::CullThreadPerCameraDrawThreadPerContext
                        };
                        const std::string modelNames[] = {
                            "SingleThreaded",
                            "DrawThreadPerContext",
                            "CullDrawThreadPerContext",
                            "CullThreadPerCameraDrawThreadPerContext"
                        };

                        auto vb = view(ri)->getViewerBase();
                        int tmi;
                        for (tmi = 0; tmi < 4; ++tmi)
                            if (models[tmi] == vb->getThreadingModel())
                                break;

                        ImGui::TableNextColumn();
                        if (ImGui::Button(modelNames[tmi].c_str())) {
                            auto new_tm = models[(tmi + 1) % 4];
                            vb->addUpdateOperation(new OneTimer([vb, new_tm]() {
                                vb->setThreadingModel(new_tm);
                                }));
                        }
                        ImGui::EndTable();
                    }
                    ImGui::Separator();

                    if (_sse == 0.0f)
                        _sse = _mapNode->getScreenSpaceError();

                    if (ImGui::SliderFloat("SSE", &_sse, 1.0f, 200.0f))
                    {
                        _mapNode->setScreenSpaceError(_sse);
                        dirtySettings();
                    }

                    ImGui::Separator();

                    if (ImGui::TreeNodeEx("Overlays", ImGuiTreeNodeFlags_DefaultOpen))
                    {
                        static int s_renderMode = 0;
                        int m = 0;

                        if (ImGui::RadioButton("Off", &s_renderMode, m++)) {
                            setRenderMode("", ri);
                        }
                        if (ImGui::RadioButton("Wireframe overlay", &s_renderMode, m++)) {
                            setRenderMode(render_outlines, ri);
                        }
                        if (ImGui::RadioButton("Front/backfacing triangles", &s_renderMode, m++)) {
                            setRenderMode(render_winding, ri);
                        }
                        if (ImGui::RadioButton("Normals (front/back)", &s_renderMode, m++)) {
                            setRenderMode(render_fb_normals, ri);
                        }
                        if (ImGui::RadioButton("Normals (view space)", &s_renderMode, m++)) {
                            setRenderMode(render_view_normals, ri);
                        }
                        if (ImGui::RadioButton("Normals (model space)", &s_renderMode, m++)) {
                            setRenderMode(render_model_normals, ri);
                        }

                        if (GLUtils::useNVGL())
                        {
                            static bool s_gpuculldebug = false;
                            if (ImGui::Checkbox("GPU cull debug view", &s_gpuculldebug)) {
                                if (s_gpuculldebug)
                                    stateset(ri)->setDefine("OE_GPUCULL_DEBUG", "1");
                                else
                                    stateset(ri)->setDefine("OE_GPUCULL_DEBUG", "0");
                            }
                        }

                        float lod_scale = camera(ri)->getLODScale();
                        if (ImGui::SliderFloat("LOD Scale", &lod_scale, 0.1f, 4.0f))
                            camera(ri)->setLODScale(lod_scale);

                        ImGui::TreePop();
                    }
                }
                ImGui::End();
            }
        };


        class NVGLInspectorGUI : public BaseGUI
        {
        public:
            NVGLInspectorGUI() : BaseGUI("NVGL Inspector")
            {
            }

            void load(const Config& conf) override
            {
            }

            void save(Config& conf) override
            {
            }

            void draw(osg::RenderInfo& ri) override
            {
                if (!isVisible())
                    return;

                ImGui::Begin(name(), visible());
                {
                    if (!GLUtils::useNVGL())
                    {
                        ImGui::TextColored(ImVec4(1, 0, 0, 1), "NVGL not enabled");
                    }
                    else
                    {
                        auto glpool = GLObjectPool::get(*ri.getState());
                        auto globjects = glpool->objects();

                        ImGui::Text("Recycling: %u%%",
                            int(100 * float(glpool->recycleHits()) / std::max(1.0f, float(glpool->recycleHits() + glpool->recycleMisses()))));
                        int avarice = glpool->getAvarice();
                        ImGui::Text("Fast"); ImGui::SameLine();
                        if (ImGui::SliderInt("Slow", &avarice, 10, 0))
                            glpool->setAvarice(avarice);

                        ImGui::Separator();

                        double glmem = (double)glpool->totalBytes() / 1048576.;
                        ImGui::TextColored(ImVec4(1, 1, 0, 1), "GL Memory: %.1lf MB", glmem);
                        std::map<std::string, std::vector<unsigned>> counts;
                        for (auto& obj : globjects) {
                            counts[obj->label()].push_back(obj->size());
                        }
                        if (ImGui::BeginTable("globjects", 2))
                        {
                            for (auto& bo : counts) {
                                unsigned total = 0;
                                for (auto& c : bo.second) total += c;
                                ImGui::TableNextColumn();
                                ImGui::Text("%s", bo.first.c_str());
                                ImGui::TableNextColumn();
                                ImGui::Text("(%d) (%.1lf MB)", bo.second.size(), (double)total / (double)1048576.);
                            }
                            ImGui::EndTable();
                        }
                    }
                }
                ImGui::End();
            }
        };
    }
}

#endif // OSGEARTH_IMGUI_SYSTEM_GUI
