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
#ifndef OSGEARTH_IMGUI_NOTIFY_GUI
#define OSGEARTH_IMGUI_NOTIFY_GUI

#include "ImGuiImp.hpp"

namespace osgEarth { namespace GUI
{
    /**
    * OSG notify handler to redirect notify messages to an ImGui text buffer
    * usage:
    *
    * ImGuiNotifyHandler* notifyHandler = new ImGuiNotifyHandler();
    * osg::setNotifyHandler(notifyHandler);
    * osgEarth::setNotifyHandler(notifyHandler);
    */
    class ImGuiNotifyHandler : public osg::NotifyHandler
    {
    public:
        void notify(osg::NotifySeverity severity, const char *message)
        {
            ScopedMutexLock lk(_mutex);
            int old_size = Buf.size();
            Buf.append(message);
            for (int new_size = Buf.size(); old_size < new_size; old_size++)
                if (Buf[old_size] == '\n')
                    LineOffsets.push_back(old_size + 1);
        }

        void Clear()
        {
            Buf.clear();
            LineOffsets.clear();
            LineOffsets.push_back(0);
        }

        Mutex _mutex;
        ImGuiTextBuffer     Buf;
        ImVector<int>       LineOffsets;        // Index to lines offset. We maintain this with AddLog() calls, allowing us to have a random access on lines
    };



    struct NotifyGUI : public BaseGUI
    {
        NotifyGUI() :
            BaseGUI("Notify Log")
        {
            _handler = dynamic_cast<ImGuiNotifyHandler*>(osg::getNotifyHandler());
        }

        void draw(osg::RenderInfo& ri) override
        {
            if (!isVisible() || _handler == nullptr)
                return;

            ImGui::Begin(name(), visible());
            {
                bool clear = ImGui::Button("Clear");
                ImGui::SameLine();
                Filter.Draw("Filter", -100.0f);

                ImGui::Separator();
                ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

                if (clear)
                    _handler->Clear();

                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
                const char* buf = _handler->Buf.begin();
                const char* buf_end = _handler->Buf.end();

                if (Filter.IsActive())
                {
                    // In this example we don't use the clipper when Filter is enabled.
                    // This is because we don't have a random access on the result on our filter.
                    // A real application processing logs with ten of thousands of entries may want to store the result of search/filter.
                    // especially if the filtering function is not trivial (e.g. reg-exp).
                    for (int line_no = 0; line_no < _handler->LineOffsets.Size; line_no++)
                    {
                        const char* line_start = buf + _handler->LineOffsets[line_no];
                        const char* line_end = (line_no + 1 < _handler->LineOffsets.Size) ? (buf + _handler->LineOffsets[line_no + 1] - 1) : buf_end;
                        if (Filter.PassFilter(line_start, line_end))
                            ImGui::TextUnformatted(line_start, line_end);
                    }
                }
                else
                {
                    // The simplest and easy way to display the entire buffer:
                    //   ImGui::TextUnformatted(buf_begin, buf_end);
                    // And it'll just work. TextUnformatted() has specialization for large blob of text and will fast-forward to skip non-visible lines.
                    // Here we instead demonstrate using the clipper to only process lines that are within the visible area.
                    // If you have tens of thousands of items and their processing cost is non-negligible, coarse clipping them on your side is recommended.
                    // Using ImGuiListClipper requires A) random access into your data, and B) items all being the  same height,
                    // both of which we can handle since we an array pointing to the beginning of each line of text.
                    // When using the filter (in the block of code above) we don't have random access into the data to display anymore, which is why we don't use the clipper.
                    // Storing or skimming through the search result would make it possible (and would be recommended if you want to search through tens of thousands of entries)
                    ImGuiListClipper clipper;
                    clipper.Begin(_handler->LineOffsets.Size);
                    while (clipper.Step())
                    {
                        for (int line_no = clipper.DisplayStart; line_no < clipper.DisplayEnd; line_no++)
                        {
                            const char* line_start = buf + _handler->LineOffsets[line_no];
                            const char* line_end = (line_no + 1 < _handler->LineOffsets.Size) ? (buf + _handler->LineOffsets[line_no + 1] - 1) : buf_end;
                            ImGui::TextUnformatted(line_start, line_end);
                        }
                    }
                    clipper.End();
                }
                ImGui::PopStyleVar();

                if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
                    ImGui::SetScrollHereY(1.0f);

                ImGui::EndChild();
                ImGui::End();
            }
        }

        ImGuiNotifyHandler* _handler;
        ImGuiTextFilter Filter;
    };
} }

#endif // OSGEARTH_IMGUI_NOTIFY_GUI
