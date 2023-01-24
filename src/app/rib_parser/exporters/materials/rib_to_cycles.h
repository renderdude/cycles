#ifndef RIB_TO_CYCLES_H
#define RIB_TO_CYCLES_H

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

class RIBtoCyclesMapping {
 public:
  using ParamMap = std::unordered_map<std::string, ustring>;
  RIBtoCyclesMapping(std::vector<std::string> nodeType, ParamMap paramMap)
      : _nodeType(nodeType), _paramMap(std::move(paramMap))
  {
  }

  ustring nodeType(int index) const
  {
    return ustring(_nodeType[index]);
  }

  virtual std::string parameter_name(const std::string &name) const
  {
    // Simple mapping case
    const auto it = _paramMap.find(name);
    return it != _paramMap.end() ? it->second.string() : name;
  }

  virtual void update_parameters(Parameter_Dictionary const &params,
                                 vector<Parsed_Parameter const *> &connections);

  virtual void add_to_graph(ShaderGraph *graph)
  {
    _nodes.back()->set_owner(graph);
    graph->add(_nodes.back());
  }

  virtual bool create_shader_node(std::string const &shader,
                                  std::string const &path,
                                  ShaderGraph *graph,
                                  Scene *scene);

  virtual ShaderNode *node(std::string name)
  {
    return _nodes.back();
  }

 protected:
  std::vector<ShaderNode *> _nodes;
  std::vector<std::string> _nodeType;
  ParamMap _paramMap;
};

class RIBtoMultiNodeCycles : public RIBtoCyclesMapping {
 public:
  RIBtoMultiNodeCycles(std::vector<std::string> nodeType,
                       ParamMap paramMap,
                       ParamMap connectionMap)
      : RIBtoCyclesMapping(nodeType, paramMap), _connectionMap(connectionMap)
  {
  }

  void add_to_graph(ShaderGraph *graph)
  {
    for (auto node : _nodes) {
      node->set_owner(graph);
      graph->add(node);
    }
  }

  virtual bool create_shader_node(std::string const &shader,
                                  std::string const &path,
                                  ShaderGraph *graph,
                                  Scene *scene);

  void update_parameters(Parameter_Dictionary const &params,
                         vector<Parsed_Parameter const *> &connections);

  ShaderNode *node(std::string name)
  {
    return _node_map[name];
  }

 protected:
  ParamMap _connectionMap;
  std::map<std::string, ShaderNode *> _node_map;
};

class PxrNormalMaptoCycles : public RIBtoMultiNodeCycles {
 public:
  using RIBtoMultiNodeCycles::RIBtoMultiNodeCycles;

  virtual bool create_shader_node(std::string const &shader,
                          std::string const &path,
                          ShaderGraph *graph,
                          Scene *scene);
};

class RIBtoCyclesTexture : public RIBtoCyclesMapping {
 public:
  using RIBtoCyclesMapping::RIBtoCyclesMapping;

  virtual bool create_shader_node(std::string const &shader,
                                  std::string const &path,
                                  ShaderGraph *graph,
                                  Scene *scene);
};

// Specializations
class PxrSurfacetoPrincipled : public RIBtoCyclesMapping {
 public:
  using RIBtoCyclesMapping::RIBtoCyclesMapping;

  void update_parameters(Parameter_Dictionary const &params,
                         vector<Parsed_Parameter const *> &connections);

 private:
  std::unordered_map<std::string, Parsed_Parameter *> _parameters;
};


CCL_NAMESPACE_END

#endif // RIB_TO_CYCLES_H
