
#ifndef __EXPORTERS_MATERIALS_H__
#define __EXPORTERS_MATERIALS_H__

#include "app/rib_parser/param_dict.h"
#include "app/rib_parser/parsed_parameter.h"
#include "scene/scene.h"
#include "scene/shader_graph.h"

#include "app/rib_parser/scene_entities.h"
#include <vector>

CCL_NAMESPACE_BEGIN

class RIBCyclesMaterials {
 public:
  RIBCyclesMaterials(Scene *scene, Mapped_Vector_Dictionary osl_shader_group)
      : _scene(scene), _osl_shader_group(osl_shader_group)
  {
  }

  ~RIBCyclesMaterials()
  {
  }

  void export_materials();

 protected:
  struct Node_Desc {
    ShaderNode *node;
    const class RIBtoCyclesMapping *mapping;
  };

  void initialize();
  void update_parameters(Node_Desc &node_desc, vector<Parsed_Parameter *> &params);
  void update_parameters();
  void update_connections(Node_Desc &node_desc,
                          ShaderGraph *shader_graph,
                          Parsed_Parameter_Vector& pv);
  void populate_shader_graph(
      std::pair<std::string, std::vector<Parameter_Dictionary>> shader_graph);

 private:
  Scene *_scene = nullptr;
  Shader *_shader = nullptr;
  Mapped_Vector_Dictionary _osl_shader_group;
  std::unordered_map<std::string, Node_Desc> _nodes;
};

CCL_NAMESPACE_END
#endif  //__EXPORTERS_MATERIALS_H__
