#include "app/rib_parser/exporters/materials/materials.h"
#include "app/rib_parser/exporters/materials/convert_lama_network.h"
#include "app/rib_parser/exporters/materials/rib_to_cycles.h"
#include "app/rib_parser/exporters/node_util.h"
#include "app/rib_parser/exporters/static_data.h"
#include "app/rib_parser/param_dict.h"
#include "app/rib_parser/parsed_parameter.h"
#include "graph/node_type.h"
#include "scene/osl.h"
#include "scene/scene.h"
#include "scene/shader_graph.h"
#include "scene/shader_nodes.h"
#include "util/log.h"
#include "util/path.h"
#include "util/string.h"
#include "util/task.h"
#include "util/vector.h"
#include <string>
#include <unordered_map>
#include <vector>

CCL_NAMESPACE_BEGIN

class RIBtoCycles {
#if 1
  const PxrSurfacetoPrincipled PxrSurface = {
      {"principled_bsdf"},
      {
          {"diffuseColor", ustring("base_color")},
          {"subsurfaceColor", ustring("subsurface_color")},
          {"subsurfaceDmfpColor", ustring("subsurface_radius")},
          {"subsurfaceIor", ustring("subsurface_ior")},
          {"subsurfaceGain", ustring("subsurface")},
          {"specularFaceColor", ustring("specular")},
          {"specularRoughness", ustring("roughness")},
          {"clearcoatRoughness", ustring("clearcoat_roughness")},
          {"glassIor", ustring("ior")},
          {"refractionGain", ustring("transmission")},
          {"glassRoughness", ustring("transmission_roughness")},
          {"glowGain", ustring("emission")},
          {"bumpNormal", ustring("normal")},
      }};
#else
  const PxrSurfacetoPrincipled PxrSurface = {
      "StandardSurfaceSR_default.oso",
      {
          // diffuse parameters
          {"diffuseGain", ustring("base")},
          {"diffuseColor", ustring("base_color")},
          {"diffuseRoughness", ustring("diffuse_roughness")},
          {"diffuseTransmitGain", ustring("transmission")},
          {"diffuseTransmitColor", ustring("transmission_color")},
          // specular parameters
          {"specularFaceColor", ustring("specular_color")},
          {"specularRoughness", ustring("specular_roughness")},
          {"specularIOR", ustring("specular_IOR")},
          {"specularAnisotropy", ustring("specular_anisotropy")},
          // clearcoat parameters
          {"clearcoatFaceColor", ustring("coat_color")},
          {"clearcoatRoughness", ustring("coat_roughness")},
          {"emissiveColor", ustring("emission_color")},
          {"opacity", ustring("alpha")},
          // Subsurface
          {"subsurfaceGain", ustring("subsurface1")},
          {"subsurfaceColor", ustring("subsurface_color")},
          {"subsurfaceDmfpColor", ustring("subsurface_radius")},
          {"subsurfaceDmfp", ustring("subsurface_scale")},
          // transmission
          {"refractionGain", ustring("transmission")},
          {"refractionColor", ustring("transmission_color")},
          {"glassIor", ustring("subsurface_radius")},
          {"glassRoughness", ustring("transmission_extra_roughness")},
          // occlusion
          // displacement
      }};
#endif

  const PxrNormalMaptoCycles PxrMultiNodeNormalMap = {
      // Nodes
      {"image_texture", "normal_map"},
      // Input Parameters
      {
          {"filename", ustring("image_texture:filename")},
          {"bumpScale", ustring("normal_map:strength")},
          {"resultN", ustring("normal_map:normal")},
      },
      // Node Connections
      {
          {"image_texture:color", ustring("normal_map:color")},
      }};

  const RIBtoCyclesMapping PxrNormalMap = {
      // Nodes
      {"normal_map"},
      // Input Parameters
      {
          {"inputRGB", ustring("color")},
          {"bumpScale", ustring("strength")},
          {"resultN", ustring("normal")},
      }};

  const RIBtoCyclesMapping PxrDefault = {{""}, {}};
  const RIBtoCyclesMapping PxrBlack = {{"diffuse_bsdf"}, {}};

  const RIBtoCyclesMapping PxrMeshLight = {{"emission"},
                                           {
                                               {"lightColor", ustring("Color")},
                                               {"strength", ustring("Strength")},
                                           }};

  const RIBtoCyclesTexture PxrTexture = {{"image_texture"},
                                         {
                                             {"filename", ustring("filename")},
                                             {"resultRGB", ustring("color")},
                                             {"resultR", ustring("color")},
                                             {"resultG", ustring("color")},
                                             {"resultB", ustring("color")},
                                             {"resultA", ustring("alpha")},
                                         }};

  const RIBtoCyclesMapping UsdPrimvarReader = {{"attribute"}, {{"varname", ustring("attribute")}}};

 public:
  RIBtoCyclesMapping *find(const std::string &nodeType, Parsed_Parameter_Vector const &pv)
  {
    RIBtoCyclesMapping *result = nullptr;

    if (nodeType == "PxrSurface") {
      result = new PxrSurfacetoPrincipled(PxrSurface);
    }
    else if (nodeType == "PxrBlack") {
      result = new RIBtoCyclesMapping(PxrBlack);
    }
    else if (nodeType == "PxrMeshLight") {
      result = new RIBtoCyclesMapping(PxrMeshLight);
    }
    else if (nodeType == "PxrTexture") {
      result = new RIBtoCyclesMapping(PxrTexture);
    }
    else if (nodeType == "PxrNormalMap") {
      bool has_texture_node = false;
      for ( auto pp: pv)
        if (pp->name == "inputRGB" && pp->storage == Container_Type::Reference) {
          has_texture_node = true;
          break;
        }
      if (has_texture_node)
        result = new RIBtoCyclesMapping(PxrNormalMap);
      else
        result = new PxrNormalMaptoCycles(PxrMultiNodeNormalMap);
    }
    else {
      result = new RIBtoCyclesMapping(PxrDefault);
    }

    return result;
  }
};

Static_Data<RIBtoCycles> sRIBtoCycles;

void RIBCyclesMaterials::export_materials()
{
  TaskPool pool;
  set<Shader *> updated_shaders;

  for (const auto &shader : _osl_shader_group) {
    initialize();
    populate_shader_graph(shader);
    add_default_renderman_inputs(_shader);

    _shader->tag_update(_scene);
    // pool.push(function_bind(&ShaderGraph::simplify, _shader->graph, _scene));
    /* NOTE: Update shaders out of the threads since those routines
     * are accessing and writing to a global context.
     */
    // updated_shaders.insert(_shader);

    _shader = nullptr;
  }

  // pool.wait_work();

  // for (Shader *shader : updated_shaders) {
  //   for (auto& node: shader->graph->nodes) {
  //     if (node->is_a(TextureCoordinateNode::node_type))
  //       for (auto& input: node->inputs)
  //       input->disconnect();
  //   }
  //   shader->tag_update(_scene);
  // }
}

void RIBCyclesMaterials::initialize()
{
  if (_shader) {
    return;
  }

  _shader = _scene->create_node<Shader>();
}

void RIBCyclesMaterials::update_connections(RIBtoCyclesMapping *mapping,
                                            ShaderGraph *shader_graph,
                                            vector<Parsed_Parameter const *> &pv)
{
  for (auto pp : pv) {
    if (pp->storage == Container_Type::Reference) {
      vector<string> tokens;
      string_split(tokens, pp->strings[0], ":");
      std::string dst_socket_name = pp->name;

      std::string input_name = mapping->parameter_name(dst_socket_name);
      std::string shader_name = "";
      if (input_name.find(":") != std::string::npos) {
        vector<string> tokens;
        string_split(tokens, input_name, ":");
        shader_name = tokens[0];
        input_name = tokens[1];
      }

      // Find the input to connect to on the passed in node
      ShaderInput *input = nullptr;
      for (ShaderInput *in : mapping->node(shader_name)->inputs) {
        if (string_iequals(in->socket_type.name.string(), input_name)) {
          input = in;
          break;
        }
      }

      if (!input) {
        fprintf(stderr,
                "Ignoring connection on '%s.%s', input '%s' was not found\n",
                mapping->node(shader_name)->name.c_str(),
                dst_socket_name.c_str(),
                input_name.c_str());
        continue;
      }

      // Now find the output to connect from
      const auto src_node_it = _nodes.find(tokens[0]);
      if (src_node_it == _nodes.end()) {
        fprintf(stderr,
                "Ignoring connection from '%s.%s' to '%s.%s', node '%s' was not found\n",
                tokens[0].c_str(),
                tokens[1].c_str(),
                mapping->node(shader_name)->name.c_str(),
                dst_socket_name.c_str(),
                tokens[0].c_str());
        continue;
      }

      const RIBtoCyclesMapping *output_mapping = src_node_it->second;
      std::string output_name = output_mapping ? output_mapping->parameter_name(tokens[1]) :
                                                 tokens[1];
      if (output_name.find(":") != std::string::npos) {
        vector<string> tokens;
        string_split(tokens, output_name, ":");
        shader_name = tokens[0];
        output_name = tokens[1];
      }

      ShaderOutput *output = nullptr;
      for (ShaderOutput *out : src_node_it->second->node(shader_name)->outputs) {
        if (string_iequals(out->socket_type.name.string(), output_name)) {
          output = out;
          break;
        }
      }

      if (!output) {
        fprintf(stderr,
                "Ignoring connection from '%s.%s' to '%s.%s', output '%s' was not found\n",
                tokens[0].c_str(),
                tokens[1].c_str(),
                mapping->node(shader_name)->name.c_str(),
                dst_socket_name.c_str(),
                output_name.c_str());
        continue;
      }

      shader_graph->connect(output, input);
    }
  }
}

void RIBCyclesMaterials::populate_shader_graph(Vector_Dictionary sg)
{
  LamaNetwork lama(sg);
  Vector_Dictionary shader_graph = lama.convert();

  std::string shader_id = shader_graph.first;
  std::string shader_name, shader_type, handle;
  std::string shader_path = path_get("shader");

  std::map<std::string, vector<Parsed_Parameter const *>> connections;
  vector<vector<Parsed_Parameter *>> terminals;

  auto graph = new ShaderGraph();

  for (auto const &params : shader_graph.second) {
    RIBtoCyclesMapping *mapping;

    auto pv = params.get_parameter_vector();
    auto pp = params.get_parameter("shader_type");
    if (pp) {
      shader_type = pp->strings[0];
      shader_name = pp->strings[1];
      handle = pp->strings[2];
      mapping = sRIBtoCycles->find(shader_name, pv);

      if (!mapping->create_shader_node(shader_name, shader_path, graph, _scene))
        continue;

      mapping->add_to_graph(graph);
      _nodes.emplace(handle, mapping);
      connections[handle].push_back(pp);
    }

    pp = params.get_parameter("__materialid");
    if (pp) {
      terminals.push_back(pv);
      _shader->name = pp->strings[0];
    }

    mapping->update_parameters(params, connections[handle]);
  }

  // Now that all nodes have been constructed, iterate the network again and build up any
  // connections between nodes
  for (auto it = connections.begin(); it != connections.end(); it++) {
    // There's an entry in connections for each node in the graph,
    // but only entries with 2 or more nodes really have a connection
    if (it->second.size() > 1) {
      for (auto pp = it->second.begin(); pp != it->second.end(); ++pp) {
        if (!(*pp)->name.compare("shader_type")) {
          shader_name = (*pp)->strings[2];
          const auto nodeIt = _nodes.find(shader_name);
          if (nodeIt == _nodes.end()) {
            fprintf(stderr, "Could not find node '%s' to connect\n", shader_name.c_str());
            continue;
          }

          update_connections(nodeIt->second, graph, it->second);
        }
      }
    }
  }

  // Finally connect the terminals to the graph output (Surface, Volume, Displacement)
  for (const auto &terminal_entry : terminals) {
    for (auto pp : terminal_entry) {
      if (!pp->name.compare("shader_type")) {
        shader_type = pp->strings[0];
        shader_name = pp->strings[1];
        handle = pp->strings[2];
        break;
      }
    }

    const auto nodeIt = _nodes.find(handle);
    ShaderNode *const node = nodeIt->second->node("");

    const char *inputName = nullptr;
    const char *outputName = nullptr;
    if (shader_type == "Displace") {
      inputName = outputName = "Displacement";
    }
    else if (shader_name == "PxrVolume") {
      inputName = outputName = "Volume";
    }
    else if (shader_name == "PxrMeshLight" && node->type->name == "emission") {
      inputName = "Surface";
      outputName = "Emission";
    }
    else {  // "PxrSurface" || "Pxr*Hair"
      inputName = "Surface";
      // Find default output name based on the node if none is provided
      if (node->type->name == "add_closure" || node->type->name == "mix_closure") {
        outputName = "Closure";
      }
      else if (node->type->name == "emission") {
        outputName = "Emission";
      }
      else {
        outputName = "BSDF";
      }
    }

    ShaderInput *const input = inputName ? graph->output()->input(inputName) : nullptr;
    if (!input) {
      fprintf(stderr, "Could not find terminal input '%s'", inputName ? inputName : "<null>");
      continue;
    }

    ShaderOutput *output = outputName ? node->output(outputName) : nullptr;
    if (!output)
      for (auto *const out : node->outputs) {
        if (out->socket_type.type == SocketType::CLOSURE) {
          output = out;
          break;
        }
      }

    if (!output) {
      fprintf(stderr,
              "Could not find terminal output '%s.%s'",
              node->name.c_str(),
              outputName ? outputName : "<null>");
      continue;
    }

    graph->connect(output, input);
  }

  _shader->set_graph(graph);
}

void RIBCyclesMaterials::add_default_renderman_inputs(Shader *shader)
{
  ShaderNode *geom = NULL;
  ShaderNode *texco = NULL;
  ShaderNode *sep_xyz = NULL;
  ShaderNode *sep_uv = NULL;
  bool found_geom = false, found_texco = false;

  auto graph = shader->graph;
  // First check if ShaderGraph::simplify added a geometry or texture coordinate node
  for (ShaderNode *node : graph->nodes) {
    if (node->is_a(GeometryNode::node_type)) {
      geom = node;
      found_geom = true;
    }
    else if (node->is_a(TextureCoordinateNode::node_type)) {
      texco = node;
      found_texco = true;
    }
  }

  if (!texco)
    texco = graph->create_node<TextureCoordinateNode>();
  if (!geom)
    geom = graph->create_node<GeometryNode>();

  using Link_Map = std::unordered_map<const NodeType *, std::pair<ShaderOutput *, std::string>>;
  Link_Map links = {
      {ImageTextureNode::node_type, {texco->output("UV"), "Vector"}},
  };

  for (auto link : links) {
    for (ShaderNode *node : graph->nodes) {
      if (node->is_a(link.first))
        graph->connect(link.second.first, node->input(link.second.second.c_str()));
    }
  }

  for (ShaderNode *node : graph->nodes) {
    bool has_s = false, has_t = false, has_st = false;
    bool has_u = false, has_v = false;
    bool has_x = false, has_y = false, has_z = false;
    for (ShaderInput *input : node->inputs) {
      if (!input->name().compare("s") || !input->name().compare("S"))
        has_s = true;
      else if (!input->name().compare("t") || !input->name().compare("T"))
        has_t = true;
      else if (!input->name().compare("st") || !input->name().compare("ST"))
        has_st = true;
      else if (!input->name().compare("u") || !input->name().compare("U"))
        has_u = true;
      else if (!input->name().compare("v") || !input->name().compare("V"))
        has_v = true;
      else if (!input->name().compare("x") || !input->name().compare("X"))
        has_x = true;
      else if (!input->name().compare("y") || !input->name().compare("Y"))
        has_y = true;
      else if (!input->name().compare("z") || !input->name().compare("Z"))
        has_z = true;
    }

    for (ShaderInput *input : node->inputs) {
      if (!input->link) {
        if (has_s && has_t) {
          if (!input->name().compare("s") || !input->name().compare("S")) {
            if (!sep_uv) {
              sep_uv = graph->create_node<SeparateXYZNode>();
              graph->connect(texco->output("UV"), sep_uv->input("Vector"));
            }

            graph->connect(sep_uv->output("X"), input);
          }
          else if (!input->name().compare("t") || !input->name().compare("T")) {
            if (!sep_uv) {
              sep_uv = graph->create_node<SeparateXYZNode>();
              graph->connect(texco->output("UV"), sep_uv->input("Vector"));
            }

            graph->connect(sep_uv->output("Y"), input);
          }
          else if (has_st) {
            if (!input->name().compare("st") || !input->name().compare("ST")) {
              graph->connect(texco->output("UV"), input);
            }
          }
          else if (has_u && has_v) {
            if (!input->name().compare("u") || !input->name().compare("U")) {
              if (!sep_uv) {
                sep_uv = graph->create_node<SeparateXYZNode>();
                graph->connect(texco->output("UV"), sep_uv->input("Vector"));
              }

              graph->connect(sep_uv->output("X"), input);
            }
            else if (!input->name().compare("v") || !input->name().compare("V")) {
            }
            if (!sep_uv) {
              sep_uv = graph->create_node<SeparateXYZNode>();
              graph->connect(texco->output("UV"), sep_uv->input("Vector"));
            }

            graph->connect(sep_uv->output("Y"), input);
          }
        }
        else if (has_x && has_y && has_z) {
          if (!input->name().compare("x") || !input->name().compare("X")) {
            if (!sep_xyz) {
              sep_xyz = graph->create_node<SeparateXYZNode>();
              graph->connect(geom->output("Position"), sep_xyz->input("Vector"));
            }

            graph->connect(sep_xyz->output("X"), input);
          }
          else if (!input->name().compare("y") || !input->name().compare("Y")) {
            if (!sep_xyz) {
              sep_xyz = graph->create_node<SeparateXYZNode>();
              graph->connect(geom->output("Position"), sep_xyz->input("Vector"));
            }

            graph->connect(sep_xyz->output("Y"), input);
          }
          else if (!input->name().compare("z") || !input->name().compare("Z")) {
            if (!sep_xyz) {
              sep_xyz = graph->create_node<SeparateXYZNode>();
              graph->connect(geom->output("Position"), sep_xyz->input("Vector"));
            }

            graph->connect(sep_xyz->output("Z"), input);
          }
        }
      }
    }
  }

  if (!found_geom)
    graph->add(geom);
  if (!found_texco)
    graph->add(texco);
  if (sep_uv)
    graph->add(sep_uv);
  if (sep_xyz)
    graph->add(sep_xyz);
}

CCL_NAMESPACE_END