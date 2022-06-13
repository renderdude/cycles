#include "app/rib_parser/exporters/materials.h"
#include "app/rib_parser/exporters/node_util.h"
#include "app/rib_parser/exporters/static_data.h"
#include "app/rib_parser/parsed_parameter.h"
#include "scene/osl.h"
#include "scene/shader_graph.h"
#include "scene/shader_nodes.h"
#include "util/path.h"
#include "util/string.h"
#include "util/task.h"

CCL_NAMESPACE_BEGIN

class RIBtoCyclesMapping {
  using ParamMap = std::unordered_map<std::string, ustring>;

 public:
  RIBtoCyclesMapping(const char *nodeType, ParamMap paramMap)
      : _nodeType(nodeType), _paramMap(std::move(paramMap))
  {
  }

  ustring nodeType() const
  {
    return _nodeType;
  }

  virtual std::string parameter_name(const std::string &name) const
  {
    // Simple mapping case
    const auto it = _paramMap.find(name);
    return it != _paramMap.end() ? it->second.string() : name;
  }

 private:
  const ustring _nodeType;
  ParamMap _paramMap;
};

#if 0
class RIBtoCyclesTexture : public RIBtoCyclesMapping {
 public:
  using RIBtoCyclesMapping::RIBtoCyclesMapping;

  std::string parameter_name(const TfToken &name,
                            const ShaderInput *inputConnection,
                            VtValue *value) const override
  {
    if (value) {
      // Remap UsdUVTexture.wrapS and UsdUVTexture.wrapT to cycles_image_texture.extension
      if (name == CyclesMaterialTokens->wrapS || name == CyclesMaterialTokens->wrapT) {
        std::string valueString = VtValue::Cast<std::string>(*value).Get<std::string>();

        // A value of 'repeat' in USD is equivalent to 'periodic' in Cycles
        if (valueString == "repeat") {
          *value = VtValue(CyclesMaterialTokens->periodic);
        }

        return "extension";
      }
    }

    return RIBtoCyclesMapping::parameter_name(name, inputConnection, value);
  }
};
#endif

class RIBtoCycles {
  const RIBtoCyclesMapping PxrSurface = {
      "principled_bsdf",
      {
          {"diffuseColor", ustring("base_color")},
          {"emissiveColor", ustring("emission")},
          {"specularColor", ustring("specular")},
          {"clearcoatRoughness", ustring("clearcoat_roughness")},
          {"opacity", ustring("alpha")},
          // opacityThreshold
          // occlusion
          // displacement
      }};

#if 0      
  const RIBtoCyclesTexture UsdUVTexture = {
      "image_texture",
      {
          {CyclesMaterialTokens->st, "vector"},
          {CyclesMaterialTokens->wrapS, "extension"},
          {CyclesMaterialTokens->wrapT, "extension"},
          {"file", "filename"},
          {"sourceColorSpace", "colorspace"},
      }};
#endif

  const RIBtoCyclesMapping UsdPrimvarReader = {"attribute", {{"varname", ustring("attribute")}}};

 public:
  const RIBtoCyclesMapping *find(const std::string &usdNodeType)
  {
    if (usdNodeType == "PxrSurface") {
      return &PxrSurface;
    }
#if 0
    if (usdNodeType == CyclesMaterialTokens->UsdUVTexture) {
      return &UsdUVTexture;
    }
    if (usdNodeType == CyclesMaterialTokens->UsdPrimvarReader_float ||
        usdNodeType == CyclesMaterialTokens->UsdPrimvarReader_float2 ||
        usdNodeType == CyclesMaterialTokens->UsdPrimvarReader_float3 ||
        usdNodeType == CyclesMaterialTokens->UsdPrimvarReader_float4 ||
        usdNodeType == CyclesMaterialTokens->UsdPrimvarReader_int) {
      return &UsdPrimvarReader;
    }
#endif

    return nullptr;
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
    //add_default_renderman_inputs(_shader);

    pool.push(function_bind(&ShaderGraph::simplify, _shader->graph, _scene));
    /* NOTE: Update shaders out of the threads since those routines
     * are accessing and writing to a global context.
     */
    updated_shaders.insert(_shader);

    _shader = nullptr;
  }

  pool.wait_work();

  for (Shader *shader : updated_shaders) {
    shader->tag_update(_scene);
  }
}

void RIBCyclesMaterials::initialize()
{
  if (_shader) {
    return;
  }

  _shader = _scene->create_node<Shader>();
}

void RIBCyclesMaterials::update_parameters(Node_Desc &node_desc,
                                           vector<Parsed_Parameter *> &parameters)
{
  if (node_desc.mapping != nullptr) {
    for (const auto param : parameters) {
      // Check if the parameter is a connection, and defer processing
      // if it is
      if (param->storage != Container_Type::Reference) {
        // See if the parameter name is in Pixar terms, and needs to be converted
        const RIBtoCyclesMapping *input_mapping = node_desc.mapping;
        const std::string input_name = input_mapping->parameter_name(param->name);

        // Find the input to write the parameter value to
        const SocketType *input = nullptr;
        for (const SocketType &socket : node_desc.node->type->inputs) {
          if (string_iequals(socket.name.string(), input_name) || socket.ui_name == input_name) {
            input = &socket;
            break;
          }
        }

        if (!input) {
          fprintf(stderr,
                  "Could not find parameter '%s' on node '%s'\n",
                  param->name.c_str(),
                  node_desc.node->name.c_str());
          continue;
        }

        set_node_value(node_desc.node, *input, param);
      }
    }
  }
}

void RIBCyclesMaterials::update_parameters()
{
}

void RIBCyclesMaterials::update_connections(Node_Desc &node_desc,
                                            ShaderGraph *shader_graph,
                                            Parsed_Parameter_Vector &pv)
{
  for (auto pp : pv) {
    if (pp->storage == Container_Type::Reference) {
      vector<string> tokens;
      string_split(tokens, pp->strings[0], ":");
      std::string dst_socket_name = pp->name;

      const RIBtoCyclesMapping *input_mapping = node_desc.mapping;
      const std::string input_name = input_mapping ?
                                         input_mapping->parameter_name(dst_socket_name) :
                                         dst_socket_name;

      // Find the input to connect to on the passed in node
      ShaderInput *input = nullptr;
      for (ShaderInput *in : node_desc.node->inputs) {
        if (string_iequals(in->socket_type.name.string(), input_name)) {
          input = in;
          break;
        }
      }

      if (!input) {
        fprintf(stderr,
                "Ignoring connection on '%s.%s', input '%s' was not found\n",
                node_desc.node->name.c_str(),
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
                node_desc.node->name.c_str(),
                dst_socket_name.c_str(),
                tokens[0].c_str());
        continue;
      }

      const RIBtoCyclesMapping *output_mapping = src_node_it->second.mapping;
      const std::string output_name = output_mapping ? output_mapping->parameter_name(tokens[1]) :
                                                       tokens[1];

      ShaderOutput *output = nullptr;
      for (ShaderOutput *out : src_node_it->second.node->outputs) {
        if (string_iequals(out->socket_type.name.string(), output_name)) {
          output = out;
          break;
        }
      }

      if (!output) {
        fprintf(stderr,
                "Ignoring connection from '%s.%s' to '%s.%s', output '%s' was not found",
                tokens[0].c_str(),
                tokens[1].c_str(),
                node_desc.node->name.c_str(),
                dst_socket_name.c_str(),
                output_name.c_str());
        continue;
      }

      shader_graph->connect(output, input);
    }
  }
}

void RIBCyclesMaterials::populate_shader_graph(
    std::pair<std::string, std::vector<Parameter_Dictionary>> shader_graph)
{
  std::string shader_id = shader_graph.first;
  std::string shader_name, shader_type, handle;
  std::string shader_path = path_get("shader");

  std::map<std::string, Parsed_Parameter_Vector> connections;
  vector<vector<Parsed_Parameter *>> terminals;

  auto graph = new ShaderGraph();

  for (auto const &params : shader_graph.second) {
    auto pv = params.get_parameter_vector();
    Node_Desc node_desc = {};

    for (auto pp : pv) {
      if (!pp->name.compare("shader_type")) {
        shader_type = pp->strings[0];
        shader_name = pp->strings[1];
        handle = pp->strings[2];
        node_desc.mapping = sRIBtoCycles->find(shader_name);

        if (node_desc.mapping) {
          if (const NodeType *nodeType = NodeType::find(node_desc.mapping->nodeType())) {
            node_desc.node = static_cast<ShaderNode *>(nodeType->create(nodeType));
          }
          else {
            fprintf(stderr, "Could not create node '%s'", shader_name.c_str());
            continue;
          }
        }
        else {
          if (string_endswith(shader_name, ".oso")) {
            if (path_is_relative(shader_name))
              shader_name = path_join(shader_path, shader_name);
            node_desc.node = OSLShaderManager::osl_node(
                graph, _scene->shader_manager, shader_name, "");
          }
          else {
            fprintf(stderr, "Could not create node '%s'", shader_name.c_str());
            continue;
          }
        }

        node_desc.node->set_owner(graph);
        graph->add(node_desc.node);
        _nodes.emplace(handle, node_desc);
        connections[handle].push_back(pp);
      }
      else if (!pp->name.compare("__materialid")) {
        terminals.push_back(pv);
        _shader->name = pp->strings[0];
      }
      else if (pp->storage == Container_Type::Reference)
        connections[handle].push_back(pp);
    }

    update_parameters(node_desc, pv);
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
    ShaderNode *const node = nodeIt->second.node;

    const char *inputName = nullptr;
    const char *outputName = nullptr;
    if (shader_name == "PxrSurface") {
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
    else if (shader_type == "Displace") {
      inputName = outputName = "Displacement";
    }
    else if (shader_name == "PxrVolume") {
      inputName = outputName = "Volume";
    }

    ShaderInput *const input = inputName ? graph->output()->input(inputName) : nullptr;
    if (!input) {
      fprintf(stderr, "Could not find terminal input '%s'", inputName ? inputName : "<null>");
      continue;
    }

    ShaderOutput *const output = outputName ? node->output(outputName) : nullptr;
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
            if (!texco)
              texco = graph->create_node<TextureCoordinateNode>();
            if (!sep_uv) {
              sep_uv = graph->create_node<SeparateXYZNode>();
              graph->connect(texco->output("UV"), sep_uv->input("Vector"));
            }

            graph->connect(sep_uv->output("X"), input);
          }
          else if (!input->name().compare("t") || !input->name().compare("T")) {
            if (!texco)
              texco = graph->create_node<TextureCoordinateNode>();
            if (!sep_uv) {
              sep_uv = graph->create_node<SeparateXYZNode>();
              graph->connect(texco->output("UV"), sep_uv->input("Vector"));
            }

            graph->connect(sep_uv->output("Y"), input);
          }
          else if (has_st) {
            if (!input->name().compare("st") || !input->name().compare("ST")) {
              if (!texco)
                texco = graph->create_node<TextureCoordinateNode>();

              graph->connect(texco->output("UV"), input);
            }
          }
          else if (has_u && has_v) {
            if (!input->name().compare("u") || !input->name().compare("U")) {
              if (!texco)
                texco = graph->create_node<TextureCoordinateNode>();
              if (!sep_uv) {
                sep_uv = graph->create_node<SeparateXYZNode>();
                graph->connect(texco->output("UV"), sep_uv->input("Vector"));
              }

              graph->connect(sep_uv->output("X"), input);
            }
            else if (!input->name().compare("v") || !input->name().compare("V")) {
            }
            if (!texco)
              texco = graph->create_node<TextureCoordinateNode>();
            if (!sep_uv) {
              sep_uv = graph->create_node<SeparateXYZNode>();
              graph->connect(texco->output("UV"), sep_uv->input("Vector"));
            }

            graph->connect(sep_uv->output("Y"), input);
          }
          else if (has_x && has_y && has_z) {
            if (!input->name().compare("x") || !input->name().compare("X")) {
              if (!geom)
                geom = graph->create_node<GeometryNode>();
              if (!sep_xyz) {
                sep_xyz = graph->create_node<SeparateXYZNode>();
                graph->connect(geom->output("Position"), sep_xyz->input("Vector"));
              }

              graph->connect(sep_xyz->output("X"), input);
            }
            else if (!input->name().compare("y") || !input->name().compare("Y")) {
              if (!geom)
                geom = graph->create_node<GeometryNode>();
              if (!sep_xyz) {
                sep_xyz = graph->create_node<SeparateXYZNode>();
                graph->connect(geom->output("Position"), sep_xyz->input("Vector"));
              }

              graph->connect(sep_xyz->output("Y"), input);
            }
            else if (!input->name().compare("z") || !input->name().compare("Z")) {
              if (!geom)
                geom = graph->create_node<GeometryNode>();
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
  }

  if (!found_geom && geom)
    graph->add(geom);
  if (!found_texco && texco)
    graph->add(texco);
  if (sep_uv)
    graph->add(sep_uv);
  if (sep_xyz)
    graph->add(sep_xyz);
}

CCL_NAMESPACE_END
