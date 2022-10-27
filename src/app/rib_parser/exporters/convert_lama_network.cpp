#include "convert_lama_network.h"

#include "app/rib_parser/error.h"
#include "app/rib_parser/param_dict.h"
#include "app/rib_parser/parsed_parameter.h"

#include "util/path.h"
#include "util/string.h"
#include "util/thread.h"

#include <OSL/oslcomp.h>
#include <OSL/oslquery.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

CCL_NAMESPACE_BEGIN

// Indent levels for prettying the output.
static std::string L0 = "";
static std::string L1 = "  ";
static std::string L2 = "    ";
static std::string L3 = "      ";
static std::string L4 = "        ";

bool LamaNetwork::generate_osl(std::string shader_name)
{
  int result = 0;
#if 1
  std::string filename = "/tmp/" + shader_name + ".mtlx";
  std::ofstream out(filename);
  out << _mtlx_def;
  out.close();

  std::string mx_root_dir = MATERIALX_ROOT_DIR;
  std::string cmd = "python3.10 " + mx_root_dir + "/bin/generateshader.py ";
  cmd += "--path " + mx_root_dir;
  cmd += " --target osl --outputPath /tmp " + filename;
  result = std::system(cmd.c_str());
  std::cout << cmd << " = " << result << std::endl;
#endif

  bool ok = false;
  if (result == 0) {
    vector<string> options;
    string stdosl_path;
    string shader_path = path_get("shader");

    /* Specify output file name. */
    options.push_back("-o");
    std::string shader_file = "/tmp/" + shader_name + "_mtlx";
    options.push_back(shader_file + ".oso");

    /* Specify standard include path. */
    string include_path_arg = string("-I") + shader_path;
    options.push_back(include_path_arg);

    stdosl_path = path_join(shader_path, "stdcycles.h");

    /* Compile.
     *
     * Mutex protected because the OSL compiler does not appear to be thread safe, see T92503. */
    static thread_mutex osl_compiler_mutex;
    thread_scoped_lock lock(osl_compiler_mutex);

    OSL::OSLCompiler *compiler = new OSL::OSLCompiler(&OSL::ErrorHandler::default_handler());
    ok = compiler->compile(string_view(shader_file + ".osl"), options, string_view(stdosl_path));
    delete compiler;
  }

  return ok;
}

std::string lama_type(Parsed_Parameter *pp)
{
  if (pp->type == "color")
    return "color3";
  else if (pp->type == "normal")
    return "vector3";
  else if (pp->type == "bxdf")
    return "BSDF";
  else if (pp->type == "int")
    return "integer";
  else {
    if (pp->elem_per_item == 1)
      return "float";
    else if (pp->elem_per_item == 3)
      return "vector3";
    else if (pp->elem_per_item == 4)
      return "vector4";
  }
}

std::string lama_default(Parsed_Parameter *pp)
{
  if (pp->type == "color")
    return "0 0 0";
  else if (pp->type == "normal")
    return "0 0 0";
  else {
    if (pp->elem_per_item == 1)
      return "0";
    else if (pp->elem_per_item == 3)
      return "0 0 0";
    else if (pp->elem_per_item == 4)
      return "0 0 0 0";
  }
}

void LamaNetwork::remove_external_nodes()
{
  // Move all of the non-Lama nodes to the new shader graph
  for (auto it = _shader_graph.second.begin(); it != _shader_graph.second.end(); it++) {
    auto shader_type = it->get_parameter("shader_type");
    if (shader_type) {
      if (shader_type->strings[1].find("Lama") == std::string::npos) {
        _lama_shader_graph.second.push_back(*it);
        _non_lama_nodes[shader_type->strings[2]] = &(*it);
      }
      else if (shader_type->strings[1].find("LamaSurface") != std::string::npos)
        _lama_surface = *it;
    }
  }

  auto it = std::remove_if(
      _shader_graph.second.begin(), _shader_graph.second.end(), [](Parameter_Dictionary &a) {
        return (a.get_parameter("shader_type")->strings[1].find("Lama") == std::string::npos ||
                a.get_parameter("shader_type")->strings[1].find("LamaSurface") !=
                    std::string::npos);
      });
  _shader_graph.second.resize(std::distance(_shader_graph.second.begin(), it));
}

void LamaNetwork::find_common_references()
{
  // Extract all external references
  for (auto &params : _shader_graph.second)
    for (auto &pp : params.get_parameter_vector())
      if (pp->name != "shader_type") {
        if (pp->storage == Container_Type::Reference) {
          vector<string> tokens;
          string_split(tokens, pp->strings[0], ":");
          if (_non_lama_nodes.find(tokens[0]) != _non_lama_nodes.end())
            _common_ext_refs[pp->name].push_back(pp);
        }
      }

  // Simplify to just the common ones
  for (auto it = _common_ext_refs.begin(); it != _common_ext_refs.end();) {
    if (it->second.size() == 1)
      it = _common_ext_refs.erase(it);
    else
      it++;
  }
}

void LamaNetwork::find_parameters()
{
  // Catagorized all Lama node references into constants, or internal/external references
  for (auto &params : _shader_graph.second) {
    auto shader_type = params.get_parameter("shader_type");
    _handle_to_params[shader_type->strings[2]] = params;
    for (auto &pp : params.get_parameter_vector()) {
      if (pp->name != "shader_type") {
        if (pp->storage != Container_Type::Reference) {
          _constants[shader_type->strings[2]].push_back(pp);
        }
        else {
          vector<string> tokens;
          string_split(tokens, pp->strings[0], ":");
          if (_non_lama_nodes.find(tokens[0]) != _non_lama_nodes.end())
            _external_references[shader_type->strings[2]].push_back(pp);
          else
            _internal_references[shader_type->strings[2]].push_back(pp);
        }
      }
    }
  }
}

void LamaNetwork::remap_parameters()
{
  // Some of the lama input parameters clash with those from OSL. MaterialX
  // will remap automatically, but we need to keep track so we manually fix them
  std::map<std::string, int> remapper;
  remapper["color"] = 1;
  remapper["normal"] = 1;
  for (auto ref : _external_references) {
    for (auto pp : ref.second) {
      if ((_common_ext_refs.find(pp->name) == _common_ext_refs.end())) {
        if (remapper.find(pp->name) != remapper.end()) {
          _remapped_params[ref.first][pp] = pp->name + "_" + std::to_string(remapper[pp->name]);
          remapper[pp->name] = remapper[pp->name] + 1;
        }
      }
    }
  }
  std::map<std::string, std::string> remap;

  for (auto ref : _common_ext_refs) {
    if (remapper.find(ref.first) == remapper.end())
      remapper[ref.first] = 1;
    for (auto pp : ref.second) {
      std::string remapped_name = pp->name + "_" + std::to_string(remapper[pp->name]);
      remapper[pp->name] = remapper[pp->name] + 1;
      if (remap.find(pp->strings[0]) == remap.end())
        remap[pp->strings[0]] = remapped_name;
      else
        remapped_name = remap[pp->strings[0]];

      _remapped_params["common"][pp] = remapped_name;
    }
  }
}

std::string LamaNetwork::generate_parameters()
{
  // Build up the strings for the nodedef and surfaceshader components
  std::string inputs = "";
  for (auto &params : _shader_graph.second) {
    auto shader_type = params.get_parameter("shader_type");
    inputs += L2 + "<!--" + shader_type->strings[1] + "-->\n";
    for (auto &pp : params.get_parameter_vector()) {
      if (pp->name != "shader_type" && pp->storage == Container_Type::Reference) {
        bool export_ref = (_common_ext_refs.find(pp->name) != _common_ext_refs.end());
        if (export_ref) {
          if (_common_ext_refs[pp->name].size() > 1) {
            auto &ref = _common_ext_refs[pp->name];
            auto p = std::find_if(ref.begin(), ref.end(), [&](Parsed_Parameter *a) {
              return a->strings[0] == pp->strings[0];
            });
            export_ref = p != ref.end();
          }
        }
        else {
          // Remove any internal references (between lama nodes) from the interface
          if (_internal_references.find(shader_type->strings[2]) != _internal_references.end())
            for (auto pp_ir : _internal_references[shader_type->strings[2]]) {
              export_ref = pp_ir == pp;
              if (export_ref)
                break;
            }
        }
        if (!export_ref) {
          std::string iface_name = remapped_name(shader_type->strings[2], pp, pp->name);
          inputs += L2 + "<input name=\"" + iface_name + "\" type=\"" + lama_type(pp);
          inputs += "\" value=\"" + lama_default(pp) + "\"/>\n";
        }
      }
    }
  }
  inputs += L2 + "<!--common-->\n";
  for (auto it = _common_ext_refs.begin(); it != _common_ext_refs.end(); it++) {
    for (auto pp : it->second) {
      std::string iface_name = remapped_name("common", pp, it->first);
      inputs += L2 + "<input name=\"" + iface_name + "\" type=\"" + lama_type(pp);
      inputs += "\" value=\"" + lama_default(pp) + "\"/>\n";
    }
  }

  return inputs;
}

// Need to split EDF nodes until layering with emission is handled properly in MaterialX
void LamaNetwork::split_nodegraph()
{
  std::vector<Parameter_Dictionary> new_nodes;

  for (auto &params : _shader_graph.second) {
    auto shader_type = params.get_parameter("shader_type");
    bool is_bsdf = true;
    bool needs_splitting = false;

    if (shader_type->strings[1] == "LamaEmission")
      is_bsdf = false;
    else if (shader_type->strings[1] == "LamaSurface") {
      std::string mat1 = params.get_parameter("materialFront")->strings[0];
      if (_handle_to_lama.find(mat1) == _handle_to_lama.end())
        params.get_parameter("materialFront")->strings[0] += "_BSDF";
    }
    else {
      if (shader_type->strings[1] == "LamaAdd" || shader_type->strings[1] == "LamaMix") {
        std::string mat1 = params.get_parameter("material1")->strings[0];
        std::string mat2 = params.get_parameter("material2")->strings[0];
        if (!(_handle_to_lama[mat1].second && _handle_to_lama[mat2].second))
          needs_splitting = true;
      }
      if (needs_splitting) {
        Parsed_Parameter_Vector new_params;
        for (auto pp : params.get_parameter_vector())
          new_params.push_back(new Parsed_Parameter(*pp));
        Parameter_Dictionary edf_dict(new_params);
        Parsed_Parameter *edf_shader_type = edf_dict.get_parameter("shader_type");
        edf_shader_type->strings[2] += "_EDF";
        shader_type->strings[2] += "_BSDF";
        std::string mat1 = params.get_parameter("material1")->strings[0];
        std::string mat2 = params.get_parameter("material2")->strings[0];

        // mat1 points to the EDF node
        if (!_handle_to_lama[mat1].second) {
          params.remove_bxdf("material1");
          if (_handle_to_lama.find(mat2) == _handle_to_lama.end())
            params.get_parameter("material2")->strings[0] += "_BSDF";
          edf_dict.remove_bxdf("material2");
          if (_handle_to_lama.find(mat1) == _handle_to_lama.end())
            edf_dict.get_parameter("material1")->strings[0] += "_EDF";
        }
        else {
          params.remove_bxdf("material2");
          if (_handle_to_lama.find(mat1) == _handle_to_lama.end())
            params.get_parameter("material1")->strings[0] += "_BSDF";
          edf_dict.remove_bxdf("material1");
          if (_handle_to_lama.find(mat2) == _handle_to_lama.end())
            edf_dict.get_parameter("material2")->strings[0] += "_EDF";
        }
        _handle_to_lama[shader_type->strings[2]] = std::make_pair(shader_type->strings[2], true);
        _handle_to_lama[edf_shader_type->strings[2]] = std::make_pair(shader_type->strings[2],
                                                                      false);
        new_nodes.push_back(edf_dict);
      }
      else
        _handle_to_lama[shader_type->strings[2]] = std::make_pair(shader_type->strings[2],
                                                                  is_bsdf);
    }
  }
  for (auto node : new_nodes)
    _shader_graph.second.push_back(node);
}

void LamaNetwork::match_renderman_definitions()
{
  // There's a few cases where the renderman definition of a parameter does not
  // match the MaterialX definition, e.g., fresnelMode is reversed
  // So, fix the ones we know about.
  for (auto &params : _shader_graph.second) {
    auto shader_type = params.get_parameter("shader_type");
    // LamaConductor
    if (shader_type->strings[1] == "LamaConductor") {
      auto it = _constants.find(shader_type->strings[2]);
      if (it == _constants.end()) {
        // No entry in _constants, so create a new one
        Parsed_Parameter *param = new Parsed_Parameter(File_Loc());
        param->type = "int";
        param->name = "fresnelMode";
        // RenderMan artistic frensel mode is 0, MaterialX it's 1
        param->add_int(1);
        params.push_back(param);
        _constants[shader_type->strings[2]].push_back(param);
      }
      else {
        bool found = false;
        for (auto pp : it->second) {
          if (pp->name == "fresnelMode") {
            pp->ints[0] = !pp->ints[0];
            found = true;
            break;
          }
        }
        // No entry, so add one
        if (!found) {
          Parsed_Parameter *param = new Parsed_Parameter(File_Loc());
          param->type = "int";
          param->name = "fresnelMode";
          // RenderMan artistic frensel mode is 0, MaterialX it's 1
          param->add_int(1);
          params.push_back(param);
          it->second.push_back(param);
        }
      }
    }
  }
}

std::string LamaNetwork::generate_nodegraph()
{
  // Now, build up the nodegraph
  std::string node_graph = L1 + "<nodegraph name=\"NG\" nodedef=\"NDInputs\">\n";
  std::string def = "";
  // Starting with constants
  for (auto it = _constants.begin(); it != _constants.end(); it++) {
    for (auto pp : it->second) {
      std::string value_t = lama_type(pp);
      def += L2 + "<constant name=\"" + it->first + "_" + pp->name + "\" type=\"" + value_t +
             "\">\n";
      def += L3 + "<input name=\"value\" type=\"" + value_t + "\" value=\"";
      std::stringstream ss;
      if (pp->floats.size() == 0)
        for (auto f : pp->ints)
          ss << f << ", ";
      else
        for (auto f : pp->floats)
          ss << f << ", ";
      std::string sss = ss.str();
      sss.erase(sss.size() - 2, 2);
      def += sss + "\" />\n";
      def += L2 + "</constant>\n";
    }
  }
  node_graph += def;

  // Now the Lama nodes
  for (auto &params : _shader_graph.second) {
    auto shader_type = params.get_parameter("shader_type");

    std::string def = "";
    def += L2 + "<" + shader_type->strings[1];
    def += " name=\"" + shader_type->strings[2];
    bool is_bsdf = _handle_to_lama[shader_type->strings[2]].second;

    def += "\" type=\"" + std::string(is_bsdf ? "BSDF" : "EDF") + "\">\n";

    for (auto &pp : params.get_parameter_vector()) {
      if (pp->name != "shader_type") {
        if (pp->storage == Container_Type::Reference) {
          vector<string> tokens;
          string_split(tokens, pp->strings[0], ":");
          int idx = tokens.size() == 1 ? 0 : 1;

          std::string lama_t = lama_type(pp);
          if (lama_t == "BSDF")
            if (!_handle_to_lama[pp->strings[0]].second)
              lama_t = "EDF";
          def += L3 + "<input name=\"" + pp->name + "\" type=\"" + lama_t;
          if (_non_lama_nodes.find(tokens[0]) == _non_lama_nodes.end())
            def += "\" nodename=\"" + tokens[idx] + "\"/>\n";
          else {
            std::string iface_name = remapped_name(shader_type->strings[2], pp, pp->name);
            def += "\" interfacename=\"" + iface_name + "\"/>\n";
          }
        }
        else {
          def += L3 + "<input name=\"" + pp->name + "\" type=\"" + lama_type(pp);
          def += "\" nodename=\"" + shader_type->strings[2] + "_" + pp->name + "\"/>\n";
        }
      }
    }
    def += L2 + "</" + shader_type->strings[1] + ">\n";
    node_graph += def;
  }

  auto output = _lama_surface.get_parameter("materialFront");
  std::string output_node =
      _handle_to_params[output->strings[0]].get_parameter("shader_type")->strings[2];
  node_graph += L2 + "<surface name=\"LamaSurface\" type=\"surfaceshader\">\n";
  node_graph += L3 + "<input name=\"bsdf\" type=\"BSDF\" nodename=\"" + output_node + "\" />\n";
  node_graph += L2 + "</surface>\n";

  node_graph += L1 + "<surfacematerial name=\"" + _shader_graph.first +
                "_mtlx\" type=\"material\">\n";
  node_graph +=
      L1 + "<input name=\"surfaceshader\" type=\"surfaceshader\" nodename=\"LamaSurface\" />\n";
  node_graph += L1 + "</surfacematerial>\n";
  node_graph += L2 + "<output name=\"out_material\" type=\"material\" nodename=\"" +
                _shader_graph.first + "_mtlx\" />\n";
  node_graph += L1 + "</nodegraph>\n";

  return node_graph;
}

void LamaNetwork::generate_mtlx_definition()
{
  _lama_shader_graph.first = _shader_graph.first;

  split_nodegraph();
  remove_external_nodes();
  find_common_references();
  find_parameters();
  remap_parameters();

  // Simplify extternal refs to just the common ones
  for (auto it = _common_ext_refs.begin(); it != _common_ext_refs.end();) {
    std::sort(it->second.begin(), it->second.end(), [](Parsed_Parameter *a, Parsed_Parameter *b) {
      return a->strings[0] == b->strings[0];
    });
    auto itt = std::unique(
        it->second.begin(), it->second.end(), [](Parsed_Parameter *a, Parsed_Parameter *b) {
          return a->strings[0] == b->strings[0];
        });
    it->second.resize(std::distance(it->second.begin(), itt));
    it++;
  }

  std::string inputs = generate_parameters();

  // Create the nodedef section
  std::string node_def = L1 + "<nodedef name=\"NDInputs\" node=\"Inputs\">\n";
  node_def += inputs;
  node_def += L2 + "<output name=\"out_material\" type=\"material\" />\n";
  node_def += L1 + "</nodedef>\n";

  // and the shader definition
  std::string surface_shader = L1 + "<Inputs name=\"" + _shader_graph.first +
                               "_mtlx\" type=\"material\">\n";
  surface_shader += inputs;
  surface_shader += L1 + "</Inputs>\n";

  match_renderman_definitions();
  std::string node_graph = generate_nodegraph();

  // Put it all together
  _mtlx_def = L0 + "<?xml version=\"1.0\"?>\n";
  _mtlx_def += L0 + "<materialx version=\"1.38\">\n";
  _mtlx_def += node_def;
  _mtlx_def += node_graph;
  _mtlx_def += surface_shader;
  _mtlx_def += L0 + "</materialx>\n";

  if (generate_osl(_shader_graph.first)) {
    Parsed_Parameter_Vector params;
    File_Loc loc;
    std::string shader_file = "/tmp/" + _shader_graph.first + "_mtlx.oso";

    Parsed_Parameter *param = new Parsed_Parameter(loc);
    param->type = "string";
    param->name = "shader_type";
    param->add_string("bxdf");
    param->add_string(shader_file);
    param->add_string(_shader_graph.first + "_mtlx");
    params.push_back(param);

    // Copy the external referneces into the new parameter set
    for (auto ref : _external_references) {
      for (auto pp : ref.second) {
        if ((_common_ext_refs.find(pp->name) == _common_ext_refs.end())) {
          pp->name = remapped_name(ref.first, pp, pp->name);
          params.push_back(pp);
        }
      }
    }

    // And the common ones
    for (auto ref : _common_ext_refs)
      for (auto pp : ref.second) {
        pp->name = remapped_name("common", pp, pp->name);
        params.push_back(pp);
      }

    // And __materialid from LamaSurface node
    param = new Parsed_Parameter(loc);
    param->type = "string";
    param->name = "__materialid";
    param->add_string(_lama_surface.get_one_string("__materialid", ""));
    params.push_back(param);

    Parameter_Dictionary dict(std::move(params));
    _lama_shader_graph.second.push_back(dict);
  }
}

Vector_Dictionary LamaNetwork::convert()
{
  bool is_materialx_graph = false;

  for (auto const &params : _shader_graph.second) {
    auto pp = params.get_parameter("shader_type");
    if (pp) {
      if (pp->strings[1].find("Lama") != std::string::npos) {
        is_materialx_graph = true;
        break;
      }
    }
  }

  if (!is_materialx_graph)
    return _shader_graph;
  else {
    generate_mtlx_definition();

    return _lama_shader_graph;
  }
}

CCL_NAMESPACE_END
