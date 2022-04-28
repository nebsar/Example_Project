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
#ifndef OSGEARTH_IMGUI_SCENE_GRAPH_GUI
#define OSGEARTH_IMGUI_SCENE_GRAPH_GUI

#include "ImGuiImp.hpp"
#include <osgEarth/AnnotationUtils>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/PagedNode>
#include <osgEarth/Viewpoint>
#include <osgEarth/ViewFitter>
#include <osgEarth/ThreeDTilesLayer>
#include <osgEarth/EarthManipulator>

#include <osg/io_utils>
#include <osg/PolygonMode>

namespace osgEarth
{
    namespace GUI
    {
        using namespace osgEarth;
        using namespace osgEarth::Util;
        
        template<typename T>
        std::string printArrayValue(const T* array)
        {
            std::stringstream buf;
            unsigned int size = array->size();
            for (unsigned i = 0; i < size; ++i)
            {
                if (i > 0)
                {
                    buf << ", ";
                }
                buf << (*array)[i];
            }
            return buf.str();
        }

        template<typename T>
        void printArrayTablePretty(const std::string& name, const T* array)
        {
            if (!array) return;
            static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_SizingFixedFit;

            const float TEXT_BASE_HEIGHT = ImGui::GetTextLineHeightWithSpacing();

            // When using ScrollX or ScrollY we need to specify a size for our table container!
            // Otherwise by default the table will fit all available space, like a BeginChild() call.
            ImGui::Text(typeid(*array).name());
            ImVec2 outer_size = ImVec2(0.0f, TEXT_BASE_HEIGHT * 8);
            if (ImGui::BeginTable(name.c_str(), 2, flags, outer_size))
            {
                ImGui::TableSetupScrollFreeze(0, 1); // Make top row always visible
                ImGui::TableSetupColumn("Index", ImGuiTableColumnFlags_None);
                ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_None);
                ImGui::TableHeadersRow();

                ImGuiListClipper clipper;
                clipper.Begin(array->size());
                while (clipper.Step())
                {
                    for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++)
                    {
                        ImGui::TableNextRow();

                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%d", i);

                        ImGui::TableSetColumnIndex(1);

                        std::stringstream val;
                        val << (*array)[i];
                        ImGui::Text(val.str().c_str());
                    }
                }
                ImGui::EndTable();
            }
        }

        inline void printArrayTable(const osg::Array* array)
        {
            switch (array->getType())
            {
            case osg::Array::ByteArrayType:
                printArrayTablePretty("Data", static_cast<const osg::ByteArray*>(array));
                break;
            case osg::Array::ShortArrayType:
                printArrayTablePretty("Data", static_cast<const osg::ShortArray*>(array));
                break;
            case osg::Array::IntArrayType:
                printArrayTablePretty("Data", static_cast<const osg::IntArray*>(array));
                break;
            case osg::Array::UByteArrayType:
                printArrayTablePretty("Data", static_cast<const osg::UByteArray*>(array));
                break;
            case osg::Array::UShortArrayType:
                printArrayTablePretty("Data", static_cast<const osg::UShortArray*>(array));
                break;
            case osg::Array::UIntArrayType:
                printArrayTablePretty("Data", static_cast<const osg::UIntArray*>(array));
                break;
            case osg::Array::FloatArrayType:
                printArrayTablePretty("Data", static_cast<const osg::FloatArray*>(array));
                break;
            case osg::Array::DoubleArrayType:
                printArrayTablePretty("Data", static_cast<const osg::DoubleArray*>(array));
                break;
            case osg::Array::Vec2bArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2bArray*>(array));
                break;
            case osg::Array::Vec3bArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3bArray*>(array));
                break;
            case osg::Array::Vec4bArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4bArray*>(array));
                break;
            case osg::Array::Vec2sArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2sArray*>(array));
                break;
            case osg::Array::Vec3sArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3sArray*>(array));
                break;
            case osg::Array::Vec4sArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4sArray*>(array));
                break;
            case osg::Array::Vec2iArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2iArray*>(array));
                break;
            case osg::Array::Vec3iArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3iArray*>(array));
                break;
            case osg::Array::Vec4iArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4iArray*>(array));
                break;
#if 0
            case osg::Array::Vec2ubArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2ubArray*>(array));
                break;
            case osg::Array::Vec3ubArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3ubArray*>(array));
                break;
#endif
            case osg::Array::Vec4ubArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4ubArray*>(array));
                break;
#if 0
            case osg::Array::Vec2usArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2usArray*>(array));
                break;
            case osg::Array::Vec3usArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3usArray*>(array));
                break;
            case osg::Array::Vec4usArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4usArray*>(array));
                break;
            case osg::Array::Vec2uiArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2uiArray*>(array));
                break;
            case osg::Array::Vec3uiArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3uiArray*>(array));
                break;
            case osg::Array::Vec4uiArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4uiArray*>(array));
                break;
#endif
            case osg::Array::Vec2ArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2Array*>(array));
                break;
            case osg::Array::Vec3ArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3Array*>(array));
                break;
            case osg::Array::Vec4ArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4Array*>(array));
                break;
            case osg::Array::Vec2dArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec2dArray*>(array));
                break;
            case osg::Array::Vec3dArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec3dArray*>(array));
                break;
            case osg::Array::Vec4dArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Vec4dArray*>(array));
                break;
#if 0
            case osg::Array::MatrixArrayType:
                printArrayTablePretty("Data", static_cast<const osg::MatrixfArray*>(array));
                break;
            case osg::Array::MatrixdArrayType:
                printArrayTablePretty("Data", static_cast<const osg::MatrixdArray*>(array));
                break;
            case osg::Array::QuatArrayType:
                printArrayTablePretty("Data", static_cast<const osg::QuatArray*>(array));
                break;
#endif
            case osg::Array::UInt64ArrayType:
                printArrayTablePretty("Data", static_cast<const osg::UInt64Array*>(array));
                break;
            case osg::Array::Int64ArrayType:
                printArrayTablePretty("Data", static_cast<const osg::Int64Array*>(array));
                break;
            default:
                ImGui::Text("Unknown array type");
                break;
            }
        }

        

        static std::string printUniformValue(osg::Uniform* uniform)
        {            
            if (uniform->getFloatArray()) return printArrayValue(uniform->getFloatArray());
            if (uniform->getDoubleArray()) return printArrayValue(uniform->getDoubleArray());
            if (uniform->getIntArray()) return printArrayValue(uniform->getIntArray());
            if (uniform->getUIntArray()) return printArrayValue(uniform->getUIntArray());
            if (uniform->getUInt64Array()) return printArrayValue(uniform->getUInt64Array());
            if (uniform->getInt64Array()) return printArrayValue(uniform->getInt64Array());

            return "";
        }


        class SceneGraphGUI : public BaseGUI
        {
        public:            

            osg::ref_ptr<osg::Node> getSelectedNode()
            {
                if (!_selectedNodePath.empty())
                {
                    return _selectedNodePath.back();
                }
                return nullptr;
            }

            const osg::RefNodePath& getSelectedNodePath()
            {
                return _selectedNodePath;
            }

            void setSelectedNodePath(const osg::NodePath& nodePath)
            {
                _selectedNodePath.clear();
                for (auto itr = nodePath.begin(); itr != nodePath.end(); ++itr)
                {
                    _selectedNodePath.push_back(*itr);
                }
                _boundsDirty = true;
            }

            struct SelectNodeHandler : public osgGA::GUIEventHandler
            {
                SceneGraphGUI* _owner;

                SelectNodeHandler(SceneGraphGUI* owner) :
                    _owner(owner)
                {
                }

                bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
                {
                    osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());

                    if (ea.getEventType() == ea.PUSH && ea.getButton() == ea.LEFT_MOUSE_BUTTON && ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)
                    {
                        float w = 5.0;
                        float h = 5.0;

                        float x = ea.getX();
                        float y = ea.getY();

                        osg::ref_ptr< osgUtil::PolytopeIntersector> picker = new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW, x - w, y - h, x + w, y + h);
                        picker->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
                        osgUtil::IntersectionVisitor iv(picker.get());
                        // This is a hack, but we set the node mask to something other than 1 << 2 which is what the "selected bounds" node mask is set to to avoid picking it.  
                        // We should rework this later into something more formal so we can add widget type nodes that aren't generally interacted with.
                        iv.setTraversalMask((1 << 1));
                        view->getCamera()->accept(iv);
                        if (picker->containsIntersections())
                        {
                            osg::NodePath nodePath = picker->getIntersections().begin()->nodePath;
                            nodePath.push_back(picker->getIntersections().begin()->drawable.get());
                            _owner->setSelectedNodePath(nodePath);

                        }
                    }
                    return false;
                }
            };

            class SceneHierarchyVisitor : public osg::NodeVisitor
            {
            public:
                SceneGraphGUI* _owner;

                SceneHierarchyVisitor(SceneGraphGUI* owner) :
                    osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
                    _owner(owner)
                {
                    setNodeMaskOverride(~0);
                }

                void apply(osg::Node& node)
                {
                    // Non groups act as leaf nodes
                    std::string label = getLabel(node);
                    ImGuiTreeNodeFlags node_flags = base_flags;
                    node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
                    if (_owner->getSelectedNode() == &node)
                    {
                        node_flags |= ImGuiTreeNodeFlags_Selected;
                    }

                    bool nodeOff = node.getNodeMask() == 0u;
                    if (nodeOff)
                    {
                        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                    }
                    ImGui::TreeNodeEx(&node, node_flags, label.c_str());
                    if (nodeOff)
                    {
                        ImGui::PopStyleColor();
                    }
                    if (ImGui::IsItemClicked())
                    {                                         
                        _owner->setSelectedNodePath(getNodePath());
                    }
                }

                void apply(osg::Group& node)
                {   
                    std::string label = getLabel(node);
                    ImGuiTreeNodeFlags node_flags = base_flags;
                    if (_owner->getSelectedNode() == &node)
                    {
                        node_flags |= ImGuiTreeNodeFlags_Selected;
                    }

                    if (isInSelectedNodePath(&node))
                    {
                        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                    }

                    bool nodeOff = node.getNodeMask() == 0u;
                    if (nodeOff)
                    {
                        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                    }

                    bool node_open = ImGui::TreeNodeEx(&node, node_flags, label.c_str());
                    if (nodeOff)
                    {
                        ImGui::PopStyleColor();
                    }
                    if (ImGui::IsItemClicked())
                    {                     
                        _owner->setSelectedNodePath(getNodePath());
                    }
                    if (node_open)
                    {
                        traverse(node);
                        ImGui::TreePop();
                    }
                }

                std::string getLabel(osg::Node& node)
                {
                    std::stringstream buf;
                    buf << node.getName() << " [" << typeid(node).name() << "]";

                    osg::Group* group = node.asGroup();
                    if (group)
                    {
                        buf << " (" << group->getNumChildren() << ")";
                    }

                    return buf.str();
                }

                bool isInSelectedNodePath(osg::Node* node)
                {
                    auto selectedNodePaths = _owner->getSelectedNodePath();                    

                    for (unsigned int i = 0; i < selectedNodePaths.size(); i++)
                    {
                        if (node == selectedNodePaths[i].get())
                        {
                            return true;
                        }
                    }
                    return false;
                }

                ImGuiTreeNodeFlags base_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
            };

            SceneGraphGUI() :
                BaseGUI("Scene Graph Inspector"),
                _installedSelectNodeHandler(false),
                _selectedBounds(0),
                _node(nullptr)
            {
            }

            void draw(osg::RenderInfo& ri) override
            {
                if (!isVisible())
                    return;

                if (ImGui::Begin(name(), visible()))
                {
                    if (_node == nullptr)
                        _node = ri.getCurrentCamera();

                    if (!_mapNode.valid())
                        _mapNode = osgEarth::findTopMostNodeOfType<MapNode>(ri.getCurrentCamera());

                    if (!_installedSelectNodeHandler)
                    {
                        auto view = dynamic_cast<osgViewer::View*>(ri.getView());
                        view->addEventHandler(new SelectNodeHandler(this));
                        _installedSelectNodeHandler = true;
                    }

                    auto size = ImGui::GetWindowSize();
                    
                    if (ImGui::CollapsingHeader("Scene"))
                    {
                        // Change the size based on whether the properties panel is 
                        ImGui::BeginChild("Scene", ImVec2(0, getSelectedNode() && _propertiesExpanded ? 0.6f * size.y : 0.90f * size.y), false, ImGuiWindowFlags_HorizontalScrollbar);
                        SceneHierarchyVisitor v(this);
                        _node->accept(v);

                        ImGui::EndChild();
                    }

                    if (getSelectedNode())
                    {
                        _propertiesExpanded = ImGui::CollapsingHeader("Properties", ImGuiTreeNodeFlags_DefaultOpen);
                        if (_propertiesExpanded)
                        {
                            ImGui::BeginChild("Properties");
                            auto view = dynamic_cast<osgViewer::View*>(ri.getView());
                            auto manip = dynamic_cast<EarthManipulator*>(view->getCameraManipulator());
                            properties(getSelectedNode(), ri, manip, _mapNode.get());
                            ImGui::EndChild();
                        }
                    }
                    ImGui::End();
                }
                // If they closed the tool deselect the node.
                if (!isVisible() && getSelectedNode())
                {
                    osg::NodePath empty;
                    setSelectedNodePath(empty);
                }

                if (_boundsDirty)
                {
                    updateBoundsDebug(_mapNode.get());
                }
            }

            void updateBoundsDebug(MapNode* mapNode)
            {
                if (_selectedBounds)
                {
                    mapNode->removeChild(_selectedBounds);
                    _selectedBounds = nullptr;
                }

                auto selectedNode = getSelectedNode();

                if (selectedNode)
                {
                    osg::RefNodePath refNodePath = getSelectedNodePath();
                    osg::NodePath nodePath;
                    for (unsigned int i = 0; i < refNodePath.size(); ++i)
                    {
                        nodePath.push_back(refNodePath[i].get());
                    }
                    
                    if (nodePath.back()->asTransform())
                    {
                        nodePath.pop_back();
                    }

                    osg::Matrixd localToWorld = osg::computeLocalToWorld(nodePath);

                    osg::Drawable* drawable = selectedNode->asDrawable();
                    if (drawable)
                    {
                        osg::BoundingBoxd bb = drawable->getBoundingBox();
                        if (bb.valid())
                        {
                            _selectedBounds = new osg::MatrixTransform;
                            _selectedBounds->setName("Bounds");
                            _selectedBounds->setMatrix(localToWorld);

                            osg::MatrixTransform* center = new osg::MatrixTransform;
                            center->addChild(AnnotationUtils::createBoundingBox(bb, Color::Yellow));
                            _selectedBounds->addChild(center);                           

                            _selectedBounds->setNodeMask(1 << 2);
                            mapNode->addChild(_selectedBounds);
                        }
                    }
                    else
                    {
                        osg::BoundingSphered bs = osg::BoundingSphered(selectedNode->getBound().center(), selectedNode->getBound().radius());
                        if (bs.radius() > 0)
                        {
                            _selectedBounds = new osg::MatrixTransform;
                            _selectedBounds->setName("Bounds");
                            _selectedBounds->setMatrix(localToWorld);

                            osg::MatrixTransform* center = new osg::MatrixTransform;
                            center->setMatrix(osg::Matrix::translate(bs.center()));
                            center->addChild(AnnotationUtils::createSphere(selectedNode->getBound().radius(), Color::Yellow));
                            _selectedBounds->addChild(center);

                            _selectedBounds->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE), 1);
                            _selectedBounds->setNodeMask(1 << 2);
                            mapNode->addChild(_selectedBounds);
                        }
                    }
                }

                _boundsDirty = false;
            }


            void properties(osg::Node* node, osg::RenderInfo& ri, EarthManipulator* manip, MapNode* mapNode)
            {
                if (manip && ImGui::Button("Zoom to"))
                {
                    osg::NodePath nodePath = node->getParentalNodePaths()[0];
                    osg::Matrixd localToWorld = osg::computeLocalToWorld(nodePath);

                    osg::BoundingSphered bs(node->getBound().center(), node->getBound().radius());
                    if (bs.valid())
                    {
                        bs.center() += localToWorld.getTrans();
                        osg::Vec3d c = bs.center();
                        double r = bs.radius();
                        const SpatialReference* mapSRS = mapNode->getMap()->getSRS();

                        std::vector<GeoPoint> points;
                        GeoPoint p;
                        p.fromWorld(mapSRS, osg::Vec3d(c.x() + r, c.y(), c.z())); points.push_back(p);
                        p.fromWorld(mapSRS, osg::Vec3d(c.x() - r, c.y(), c.z())); points.push_back(p);
                        p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y() + r, c.z())); points.push_back(p);
                        p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y() - r, c.z())); points.push_back(p);
                        p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y(), c.z() + r)); points.push_back(p);
                        p.fromWorld(mapSRS, osg::Vec3d(c.x(), c.y(), c.z() - r)); points.push_back(p);

                        ViewFitter fitter(mapNode->getMap()->getSRS(), ri.getCurrentCamera());
                        Viewpoint vp;
                        if (fitter.createViewpoint(points, vp))
                        {
                            manip->setViewpoint(vp, 2.0);
                        }
                    }
                }

                bool visible = node->getNodeMask() != 0;
                if (ImGui::Checkbox("Visible", &visible))
                {
                    node->setNodeMask(visible);
                }

                if (ImGui::TreeNode("General"))
                {
                    ImGui::Text("Type %s", typeid(*node).name());
                    ImGui::Text("Name %s", node->getName().c_str());

                    osg::Drawable* drawable = node->asDrawable();
                    if (drawable)
                    {                        
                        osg::BoundingBox bbox = drawable->getBoundingBox();
                        double width = bbox.xMax() - bbox.xMin();
                        double depth = bbox.yMax() - bbox.yMin();
                        double height = bbox.zMax() - bbox.zMin();

                        ImGui::Text("Bounding box (local) center=(%.3f, %.3f, %.3f) dimensions=%.3f x %.3f %.3f", bbox.center().x(), bbox.center().y(), bbox.center().z(), width, depth, height);
                    }
                    else
                    {
                        osg::BoundingSphere bs = node->getBound();
                        ImGui::Text("Bounding sphere (local) center=(%.3f, %.3f, %.3f) radius=%.3f", bs.center().x(), bs.center().y(), bs.center().z(), bs.radius());
                    }

                    ImGui::Text("Reference count %d", node->referenceCount());

                    ImGui::TreePop();
                }

                osg::MatrixTransform* matrixTransform = dynamic_cast<osg::MatrixTransform*>(node);
                if (matrixTransform)
                {
                    if (ImGui::TreeNode("Transform"))
                    {
                        bool dirty = false;
                        osg::Matrix matrix = matrixTransform->getMatrix();
                        osg::Vec3d translate = matrixTransform->getMatrix().getTrans();
                        if (ImGui::InputScalarN("Translation", ImGuiDataType_Double, translate._v, 3))
                        {
                            dirty = true;
                        }

                        osg::Vec3d scale = matrixTransform->getMatrix().getScale();
                        if (ImGui::InputScalarN("Scale", ImGuiDataType_Double, scale._v, 3))
                        {
                            dirty = true;
                        }
                        osg::Quat rot = matrixTransform->getMatrix().getRotate();
                        if (ImGui::InputScalarN("Rotation", ImGuiDataType_Double, rot._v, 4))
                        {
                            dirty = true;
                        }

                        if (dirty)
                        {
                            osg::Matrix newMatrix = osg::Matrix::translate(translate) * osg::Matrix::rotate(rot) * osg::Matrixd::scale(scale);
                            matrixTransform->setMatrix(newMatrix);
                        }
                        ImGui::TreePop();
                    }
                }

                osgEarth::PagedNode2* pagedNode2 = dynamic_cast<osgEarth::PagedNode2*>(node);
                if (pagedNode2)
                {
                    if (ImGui::TreeNode("PagedNode"))
                    {
                        ImGui::Text("Min range %.3f", pagedNode2->getMinRange());
                        ImGui::Text("Max range %.3f", pagedNode2->getMaxRange());
                        ImGui::TreePop();
                    }
                }

                osgEarth::Contrib::ThreeDTiles::ThreeDTileNode* threeDTiles = dynamic_cast<osgEarth::Contrib::ThreeDTiles::ThreeDTileNode*>(node);
                if (threeDTiles)
                {
                    if (ImGui::TreeNode("3D Tiles"))
                    {                       
                        std::string content = threeDTiles->getTile()->content()->getJSON().toStyledString();
                        ImGui::Text("Content");
                        ImGui::TextWrapped(content.c_str());
                    }
                }

                osg::StateSet* stateset = node->getStateSet();
                if (stateset)
                {
                    if (ImGui::TreeNode("State Set"))
                    {
                        if (!stateset->getTextureAttributeList().empty() && ImGui::TreeNode("Textures"))
                        {
                            unsigned int textureWidth = 50;
                            for (unsigned int i = 0; i < stateset->getTextureAttributeList().size(); ++i)
                            {
                                osg::Texture2D* texture = dynamic_cast<osg::Texture2D*>(stateset->getTextureAttribute(i, osg::StateAttribute::TEXTURE));

                                if (texture)
                                {
                                    ImGui::PushID(texture);
                                    ImGui::BeginGroup();
                                    ImGui::Text("Unit %d", i);
                                    ImGui::Text("Name %s", texture->getName().c_str());
                                    ImGuiUtil::Texture(texture, ri, textureWidth);
                                    ImGui::EndGroup();
                                }
                            }
                            ImGui::TreePop();
                        }

                        VirtualProgram* virtualProgram = VirtualProgram::get(stateset);
                        if (virtualProgram && ImGui::TreeNode("Shaders"))
                        {
                            VirtualProgram::ShaderMap shaderMap;
                            virtualProgram->getShaderMap(shaderMap);
                            for (auto& s : shaderMap)
                            {
                                ImGui::Text("Name %s", s.second._shader->getName().c_str());
                                ImGui::Text("Location %s", FunctionLocationToString(s.second._shader->getLocation()));
                                ImGui::TextWrapped(s.second._shader->getShaderSource().c_str());
                                ImGui::Separator();
                            }
                            ImGui::TreePop();
                        }

                        if (!stateset->getUniformList().empty())
                        {
                            if (ImGui::TreeNode("Uniforms"))
                            {       
                                if (ImGui::BeginTable("Uniforms", 3, ImGuiTableFlags_Borders))
                                {
                                    for (auto& u : stateset->getUniformList())
                                    {

                                        osg::Uniform* uniform = dynamic_cast<osg::Uniform*>(u.second.first.get());
                                        if (uniform)
                                        {
                                            // Name
                                            ImGui::TableNextColumn(); ImGui::Text(u.first.c_str());
                                            // Type
                                            ImGui::TableNextColumn(); ImGui::Text(osg::Uniform::getTypename(uniform->getType()));
                                            // Value
                                            ImGui::TableNextColumn(); ImGui::Text(printUniformValue(uniform).c_str());
                                        }
                                    }
                                    ImGui::EndTable();
                                }
                            }
                        }                        

                        ImGui::TreePop();
                    }
                }

                osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(node);
                if (geometry && ImGui::TreeNode("Geometry Properties"))
                {
                    if (geometry->getVertexArray())
                    {
                        if (ImGui::TreeNode("verts", "VERTICES %d", geometry->getVertexArray()->getNumElements()))
                        {                            
                            printArrayTable(geometry->getVertexArray());
                            ImGui::TreePop();
                        }
                    }

                    if (geometry->getColorArray())
                    {
                        if (ImGui::TreeNode("colors", "COLORS %d", geometry->getColorArray()->getNumElements()))
                        {
                            printArrayTable(geometry->getColorArray());
                            ImGui::TreePop();
                        }
                    }

                    if (geometry->getNormalArray())
                    {
                        if (ImGui::TreeNode("normals", "NORMALS %d", geometry->getNormalArray()->getNumElements()))
                        {
                            printArrayTable(geometry->getNormalArray());
                            ImGui::TreePop();
                        }
                    }

                    // Vetex attrib arrays
                    osg::Geometry::ArrayList& arrays = geometry->getVertexAttribArrayList();
                    bool validVertexAttributeArray = false;
                    for (unsigned int i = 0; i < arrays.size(); ++i)
                    {
                        if (arrays[i].valid())
                        {
                            validVertexAttributeArray = true;
                            break;
                        }
                    }

                    if (validVertexAttributeArray)
                    {

                        ImGui::Text("Vertex Attributes");
                        ImGui::Separator();
                        for (unsigned int i = 0; i < arrays.size(); ++i)
                        {
                            if (!arrays[i].valid()) continue;

                            const char* arrayName = "";

                            switch (i)
                            {
                            case osg::Drawable::VERTICES:
                                arrayName = "VERTICES";
                                break;
                            case osg::Drawable::WEIGHTS:
                                arrayName = "WEIGHTS";
                                break;
                            case osg::Drawable::NORMALS:
                                arrayName = "NORMALS";
                                break;
                            case osg::Drawable::COLORS:
                                arrayName = "COLORS";
                                break;
                            case osg::Drawable::SECONDARY_COLORS:
                                arrayName = "SECONDARY_COLORS";
                                break;
                            case osg::Drawable::FOG_COORDS:
                                arrayName = "FOG_COORDS";
                                break;
                            case osg::Drawable::ATTRIBUTE_6:
                                arrayName = "ATTRIBUTE_6";
                                break;
                            case osg::Drawable::ATTRIBUTE_7:
                                arrayName = "ATTRIBUTE_7";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_0:
                                arrayName = "TEXTURE_COORDS_0";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_1:
                                arrayName = "TEXTURE_COORDS_1";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_2:
                                arrayName = "TEXTURE_COORDS_2";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_3:
                                arrayName = "TEXTURE_COORDS_3";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_4:
                                arrayName = "TEXTURE_COORDS_4";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_5:
                                arrayName = "TEXTURE_COORDS_5";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_6:
                                arrayName = "TEXTURE_COORDS_6";
                                break;
                            case osg::Drawable::TEXTURE_COORDS_7:
                                arrayName = "TEXTURE_COORDS_7";
                                break;
                            default:
                                arrayName = "Uknown Array";
                            }

                            if (ImGui::TreeNode(arrayName, "%s %d", arrayName, arrays[i]->getNumElements()))
                            {
                                printArrayTable(arrays[i].get());
                                ImGui::TreePop();
                            }
                        }
                    }

                    for (auto &p : geometry->getPrimitiveSetList())
                    {
                        ImGui::Text("%s Mode=%s Primitives=%d Instances=%d", typeid(*p.get()).name(), GLModeToString(p->getMode()), p->getNumPrimitives(), p->getNumInstances());
                    }
                }
            }

            const char* GLModeToString(GLenum mode)
            {
                switch (mode)
                {
                case(GL_POINTS): return "GL_POINTS";
                case(GL_LINES): return "GL_LINES";
                case(GL_TRIANGLES): return "GL_TRIANGLES";
                case(GL_QUADS): return "GL_QUADS";
                case(GL_LINE_STRIP): return "GL_LINE_STRIP";
                case(GL_LINE_LOOP): return "GL_LINE_LOOP";
                case(GL_TRIANGLE_STRIP): return "GL_TRIANGLE_STRIP";
                case(GL_TRIANGLE_FAN): return "GL_TRIANGLE_FAN";
                case(GL_QUAD_STRIP): return "GL_QUAD_STRIP";
                case(GL_PATCHES): return "GL_PATCHES";
                case(GL_POLYGON): return "GL_POLYGON";
                default: return "Unknown";
                }
            }

            const char* FunctionLocationToString(VirtualProgram::FunctionLocation location)
            {
                using namespace osgEarth::ShaderComp;

                switch (location)
                {
                case LOCATION_VERTEX_MODEL: return "LOCATION_VERTEX_MODEL";
                case LOCATION_VERTEX_VIEW: return "LOCATION_VERTEX_VIEW";
                case LOCATION_VERTEX_CLIP: return "LOCATION_VERTEX_CLIP";
                case LOCATION_TESS_CONTROL: return "LOCATION_TESS_CONTROL";
                case LOCATION_TESS_EVALUATION: return "LOCATION_TESS_EVALUATION";
                case LOCATION_GEOMETRY: return "LOCATION_GEOMETRY";
                case LOCATION_FRAGMENT_COLORING: return "LOCATION_FRAGMENT_COLORING";
                case LOCATION_FRAGMENT_LIGHTING: return "LOCATION_FRAGMENT_LIGHTING";
                case LOCATION_FRAGMENT_OUTPUT: return "LOCATION_FRAGMENT_OUTPUT";
                case LOCATION_UNDEFINED: return "LOCATION_UNDEFINED";
                default: return "Undefined";
                }
            }

            bool _installedSelectNodeHandler;
            osg::MatrixTransform* _selectedBounds;
            osg::observer_ptr<osg::Node> _node;
            osg::observer_ptr<MapNode> _mapNode;
            osg::RefNodePath _selectedNodePath;
            bool _propertiesExpanded = true;
            bool _boundsDirty = false;
        };
    }
}

#endif // OSGEARTH_IMGUI_SCENE_GRAPH_GUI
