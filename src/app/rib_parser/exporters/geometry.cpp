#include "app/rib_parser/parsed_parameter.h"
#include "scene/mesh.h"
#include "scene/object.h"
#include "scene/shader_graph.h"
#include "scene/shader_nodes.h"

#include "app/rib_parser/exporters/attribute.h"
#include "app/rib_parser/exporters/geometry.h"
#include "util/vector.h"
#include <cstdio>
#include <opensubdiv/vtr/types.h>
#include <string>
#include <unordered_map>

CCL_NAMESPACE_BEGIN

void RIBCyclesMesh::export_geometry()
{
  auto &shape = _inst_def->shapes[0];

  if (_inst_def->shapes.size() > 1) {
    fprintf(stderr,
            "An instance definition, %s, contains more than one shape.\n",
            _inst_def->name.c_str());
    fprintf(stderr, "Only using the first shape found.\n");
  }

  _instances.reserve(_inst_v.size());

  for (size_t i = 0; i < _inst_v.size(); ++i) {
    std::string material_id = _inst_v[i].material_name;
    if (_instanced_geom.find(material_id) != _instanced_geom.end())
      _geom = _instanced_geom[material_id];
    else {
      initialize(material_id);

      array<Node *> usedShaders(1);
      usedShaders[0] = _scene->default_surface;

      for (auto shader : _scene->shaders) {
        if (!shader->name.compare(material_id)) {
          usedShaders[0] = shader;
          break;
        }
      }

      for (Node *shader : usedShaders) {
        static_cast<Shader *>(shader)->tag_used(_scene);
      }

      _geom->set_used_shaders(usedShaders);

      // Must happen after material ID update, so that attribute decisions can be made
      // based on it (e.g. check whether an attribute is actually needed)
      bool rebuild = false;
      populate(rebuild);

      if (_geom->is_modified() || rebuild) {
        _geom->tag_update(_scene, rebuild);
        _geom->compute_bounds();
      }
    }

    _instances[i] = _scene->create_node<Object>();
    initialize_instance(static_cast<int>(i));

    if (i == 0) {
      std::string instance_id = _inst_v[0].parameters.at("identifier").get_one_string("name", "");
      // Make sure the first object attribute is the instanceId
      assert(_instances[0]->attributes.size() >= 1 &&
             _instances[0]->attributes.front().name() == instance_id);

      if (_inst_v.size() > 1) {
        _instances[0]->attributes.front() = ParamValue(instance_id, +0.0f);
      }
      else {
        // Default to a single instance with an identity transform
        _instances[0]->attributes.front() = ParamValue(instance_id, -1.0f);
      }
    }
    // Update transform
    const float metersPerUnit = 1.;

    const Transform tfm = transform_scale(make_float3(metersPerUnit)) *
                          projection_to_transform((*_inst_v[i].render_from_instance) *
                                                  (*shape.render_from_object));
    _instances[i]->set_tfm(tfm);

    /* Not sure where to pull visibility from a RIB file
    if (HdChangeTracker::IsVisibilityDirty(*dirtyBits, id)) {
      for (Object *instance : _instances) {
        instance->set_visibility(Base::IsVisible() ? ~0 : 0);
      }
    }
  */
  }
  for (Object *instance : _instances) {
    instance->tag_update(_scene);
    instance->compute_bounds(instance->use_motion());
    _bounds.grow(instance->bounds);
  }
}

void RIBCyclesMesh::initialize(std::string material_name)
{
  // Create geometry
  _geom = _scene->create_node<Mesh>();
  _geom->name = _inst_def->name;

  vector<int3> tmp_vec;
  triangles.swap(tmp_vec);

  _instanced_geom[material_name] = _geom;
}

void RIBCyclesMesh::initialize_instance(int index)
{
  Object *instance = _instances[index];
  instance->set_geometry(_geom);

  std::string id = _inst_v[index].parameters.at("identifier").get_one_string("name", "");
  instance->attributes.emplace_back(id,
                                    _instances.size() == 1 ? -1.0f : static_cast<float>(index));
  instance->set_color(make_float3(0.8f, 0.8f, 0.8f));
  instance->set_random_id(hash_uint2(hash_string(_geom->name.c_str()), index));
}

void RIBCyclesMesh::populate(bool &rebuild)
{
  populate_topology();
  populate_points();

  // Must happen after topology update, so that normals attribute size can be calculated
  populate_normals();

  // Must happen after topology update, so that appropriate attribute set can be selected
  populate_primvars();

  rebuild = (_geom->triangles_is_modified()) || (_geom->subd_start_corner_is_modified()) ||
            (_geom->subd_num_corners_is_modified()) || (_geom->subd_shader_is_modified()) ||
            (_geom->subd_smooth_is_modified()) || (_geom->subd_ptex_offset_is_modified()) ||
            (_geom->subd_face_corners_is_modified());
}

void RIBCyclesMesh::populate_normals()
{
  _geom->attributes.remove(ATTR_STD_FACE_NORMAL);
  _geom->attributes.remove(ATTR_STD_VERTEX_NORMAL);

  // Authored normals should only exist on triangle meshes
  if (_geom->get_subdivision_type() != Mesh::SUBDIVISION_NONE) {
    return;
  }

  auto &shape = _inst_def->shapes[0];
  Parsed_Parameter const *param = shape.parameters.get_parameter("N");

  if (param == nullptr)
    return;  // Ignore missing normals

  auto normals = shape.parameters.get_normal_array("N");
  float orientation = shape.reverse_orientation ? -1.0f : 1.0f;
  Container_Type interpolation = param->storage;

  if (interpolation == Container_Type::Constant) {
    const float3 constantNormal = normals[0];

    float3 *const N = _geom->attributes.add(ATTR_STD_VERTEX_NORMAL)->data_float3();
    for (size_t i = 0; i < _geom->get_verts().size(); ++i) {
      N[i] = orientation * constantNormal;
    }
  }
  else if (interpolation == Container_Type::Uniform) {
    float3 *const N = _geom->attributes.add(ATTR_STD_FACE_NORMAL)->data_float3();
    for (size_t i = 0; i < _geom->num_triangles(); ++i) {
      N[i] = orientation * normals[i / 2];
    }
  }
  else if (interpolation == Container_Type::Vertex || interpolation == Container_Type::Varying) {
    float3 *const N = _geom->attributes.add(ATTR_STD_VERTEX_NORMAL)->data_float3();
    for (size_t i = 0; i < _geom->get_verts().size(); ++i) {
      N[i] = orientation * normals[i];
    }
  }
  else if (interpolation == Container_Type::FaceVarying) {
    // Cycles has no standard attribute for face-varying normals, so this is a lossy transformation
    float3 *const N = _geom->attributes.add(ATTR_STD_FACE_NORMAL)->data_float3();
    int index_offset = 0;
    const vector<int> vertCounts = shape.parameters.get_int_array("nvertices");
    for (size_t i = 0; i < vertCounts.size(); i++) {
      for (int j = 0; j < vertCounts[i] - 2; j++) {
        int v0 = index_offset;
        int v1 = index_offset + j + 1;
        int v2 = index_offset + j + 2;

        float3 average_normal = normals[v0] + normals[v1] + normals[v2];
        N[i] = orientation * normalize(average_normal);
      }

      index_offset += vertCounts[i];
    }
  }
}

void RIBCyclesMesh::populate_primvars()
{
  Scene *const scene = (Scene *)_geom->get_owner();

  const bool subdivision = _geom->get_subdivision_type() != Mesh::SUBDIVISION_NONE;
  AttributeSet &attributes = subdivision ? _geom->subd_attributes : _geom->attributes;

  std::unordered_map<Container_Type, AttributeElement> interpolations = {
      {std::make_pair(Container_Type::FaceVarying, ATTR_ELEMENT_CORNER)},
      {std::make_pair(Container_Type::Uniform, ATTR_ELEMENT_FACE)},
      {std::make_pair(Container_Type::Vertex, ATTR_ELEMENT_VERTEX)},
      {std::make_pair(Container_Type::Varying, ATTR_ELEMENT_VERTEX)},
      {std::make_pair(Container_Type::Constant, ATTR_ELEMENT_OBJECT)},
  };

  auto &shape = _inst_def->shapes[0];
  Parsed_Parameter_Vector const &paramv = shape.parameters.get_parameter_vector();

  for (const auto param : paramv) {
    // Skip special primvars that are handled separately
    if (param->name == "P" || param->name == "N" || param->name == "nfaces" ||
        param->name == "nvertices" || param->name == "vertices") {
      continue;
    }

    const ustring name(param->name);
    AttributeStandard std = ATTR_STD_NONE;
    if (param->name == "st" || param->name == "uv") {
      std = ATTR_STD_UV;
    }
    else if (param->storage == Container_Type::Vertex) {
      if (param->name == "color") {
        std = ATTR_STD_VERTEX_COLOR;
      }
      else if (param->name == "N") {
        std = ATTR_STD_VERTEX_NORMAL;
      }
    }
    /*
    else if (desc.name == HdTokens->displayColor &&
             interpolation.first == HdInterpolationConstant) {
      if (value.IsHolding<VtVec3fArray>() && value.GetArraySize() == 1) {
        const GfVec3f color = value.UncheckedGet<VtVec3fArray>()[0];
        _instances[0]->set_color(make_float3(color[0], color[1], color[2]));
      }
    }
    */

    Parsed_Parameter *result = param;
    // Skip attributes that are not needed
    if ((std != ATTR_STD_NONE && _geom->need_attribute(scene, std)) ||
        _geom->need_attribute(scene, name)) {

      if (!subdivision) {
        // Adjust attributes for polygons that were triangulated
        if (param->storage == Container_Type::Uniform) {
          result = compute_triangulated_uniform_primvar(param);
          if (!result) {
            continue;
          }
        }
        else if (param->storage == Container_Type::FaceVarying) {
          result = compute_triangulated_face_varying_primvar(param);
          if (!result) {
            continue;
          }
        }
      }

      apply_primvars(attributes, name, result, interpolations[param->storage], std);
    }
  }
}

void RIBCyclesMesh::populate_points()
{
  auto &shape = _inst_def->shapes[0];
  auto points = shape.parameters.get_point3_array("P");
  array<float3> P_array;
  P_array = points;

  _geom->set_verts(P_array);
}

void RIBCyclesMesh::populate_topology()
{
  // Clear geometry before populating it again with updated topology
  _geom->clear(true);

  /* Get RIB refinement level
    const HdDisplayStyle displayStyle = GetDisplayStyle(sceneDelegate);
    _topology = HdMeshTopology(GetMeshTopology(sceneDelegate), displayStyle.refineLevel);
  */

  auto &shape = _inst_def->shapes[0];
  std::string subdivScheme = shape.parameters.get_one_string("scheme", "");
  if (subdivScheme == "bilinear") {
    _geom->set_subdivision_type(Mesh::SUBDIVISION_LINEAR);
  }
  else if (subdivScheme == "catmull-clark") {
    _geom->set_subdivision_type(Mesh::SUBDIVISION_CATMULL_CLARK);
  }
  else {
    _geom->set_subdivision_type(Mesh::SUBDIVISION_NONE);
  }

  const bool smooth = shape.parameters.get_one_bool("smooth", false);
  const bool subdivision = _geom->get_subdivision_type() != Mesh::SUBDIVISION_NONE;

  int shader = 0;
  const vector<int> vertIndx = shape.parameters.get_int_array("vertices");
  const vector<int> vertCounts = shape.parameters.get_int_array("nvertices");

  if (!subdivision) {
    compute_triangle_indices(vertIndx, vertCounts, triangles);

    auto points = shape.parameters.get_point3_array("P");
    _geom->reserve_mesh(points.size(), triangles.size());

    for (size_t i = 0; i < triangles.size(); ++i) {
      const int3 triangle = triangles[i];
      _geom->add_triangle(triangle[0], triangle[1], triangle[2], shader, smooth);
    }
  }
  else {
    /* TODO: subdivision meshes

    PxOsdSubdivTags subdivTags = GetSubdivTags(sceneDelegate);
    _topology.SetSubdivTags(subdivTags);

    size_t numNgons = 0;
    size_t numCorners = 0;
    for (int vertCount : vertCounts) {
      numNgons += (vertCount == 4) ? 0 : 1;
      numCorners += vertCount;
    }

    _geom->reserve_subd_faces(_topology.GetNumFaces(), numNgons, numCorners);

    // TODO: Handle hole indices
    size_t faceIndex = 0;
    size_t indexOffset = 0;
    for (int vertCount : vertCounts) {
      _geom->add_subd_face(&vertIndx[indexOffset], vertCount, faceShaders[faceIndex], smooth);

      faceIndex++;
      indexOffset += vertCount;
    }

    const VtIntArray creaseLengths = subdivTags.GetCreaseLengths();
    if (!creaseLengths.empty()) {
      size_t numCreases = 0;
      for (int creaseLength : creaseLengths) {
        numCreases += creaseLength - 1;
      }

      _geom->reserve_subd_creases(numCreases);

      const VtIntArray creaseIndices = subdivTags.GetCreaseIndices();
      const VtFloatArray creaseWeights = subdivTags.GetCreaseWeights();

      indexOffset = 0;
      size_t creaseLengthOffset = 0;
      size_t createWeightOffset = 0;
      for (int creaseLength : creaseLengths) {
        for (int j = 0; j < creaseLength - 1; ++j, ++createWeightOffset) {
          const int v0 = creaseIndices[indexOffset + j];
          const int v1 = creaseIndices[indexOffset + j + 1];

          float weight = creaseWeights.size() == creaseLengths.size() ?
                             creaseWeights[creaseLengthOffset] :
                             creaseWeights[createWeightOffset];

          _geom->add_edge_crease(v0, v1, weight);
        }

        indexOffset += creaseLength;
        creaseLengthOffset++;
      }

      const VtIntArray cornerIndices = subdivTags.GetCornerIndices();
      const VtFloatArray cornerWeights = subdivTags.GetCornerWeights();

      for (size_t i = 0; i < cornerIndices.size(); ++i) {
        _geom->add_vertex_crease(cornerIndices[i], cornerWeights[i]);
      }
    }

    _geom->set_subd_dicing_rate(1.0f);
    _geom->set_subd_max_level(_topology.GetRefineLevel());
    _geom->set_subd_objecttoworld(_instances[0]->get_tfm());
    */
  }
}

void RIBCyclesMesh::populate_shader_graph(bool initializing)
{
}

void RIBCyclesMesh::compute_triangle_indices(const vector<int> &vertices,
                                             const vector<int> &nvertices,
                                             vector<int3> &indices)
{
  int index_offset = 0;

  for (size_t i = 0; i < nvertices.size(); i++) {
    for (int j = 0; j < nvertices[i] - 2; j++) {
      int v0 = vertices[index_offset];
      int v1 = vertices[index_offset + j + 1];
      int v2 = vertices[index_offset + j + 2];

      // Reverse orientation for cycles
      indices.push_back(make_int3(v0, v1, v2));
    }

    index_offset += nvertices[i];
  }
}

Parsed_Parameter *RIBCyclesMesh::compute_triangulated_uniform_primvar(
    const Parsed_Parameter *param)
{
  Parsed_Parameter* result = new Parsed_Parameter(*param);
  vector<float> tmp;
  result->floats.swap(tmp);

  auto &shape = _inst_def->shapes[0];
  const vector<int> nvertices = shape.parameters.get_int_array("nvertices");

  int index_offset = 0;
  for (size_t i = 0; i < nvertices.size(); i++) {
    for (int j = 0; j < nvertices[i] - 2; j++) {
      result->floats.push_back(param->floats[index_offset]);
    }
    index_offset++;
  }

  return result;
}

Parsed_Parameter *RIBCyclesMesh::compute_triangulated_face_varying_primvar(
    const Parsed_Parameter *param)
{
  Parsed_Parameter* result = new Parsed_Parameter(*param);
  vector<float> tmp;
  result->floats.swap(tmp);

  auto &shape = _inst_def->shapes[0];
  const vector<int> nvertices = shape.parameters.get_int_array("nvertices");

  int index_offset = 0;
  int elem_per_item = param->elem_per_item;

  for (size_t i = 0; i < nvertices.size(); i++) {
    for (int j = 0; j < nvertices[i] - 2; j++) {
      int ind = index_offset;
      for (int k = 0; k < elem_per_item; ++k)
        result->floats.push_back(param->floats[elem_per_item * ind + k]);
      ind = index_offset + j + 1;
      for (int k = 0; k < elem_per_item; ++k)
        result->floats.push_back(param->floats[elem_per_item * ind + k]);
      ind = index_offset + j + 2;
      for (int k = 0; k < elem_per_item; ++k)
        result->floats.push_back(param->floats[elem_per_item * ind + k]);
    }

    index_offset += nvertices[i];
  }

  return result;
}

CCL_NAMESPACE_END
