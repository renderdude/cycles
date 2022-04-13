
#ifndef __EXPORTERS_GEOMETRY_H__
#define __EXPORTERS_GEOMETRY_H__

#include "scene/scene.h"

#include "app/rib_parser/scene_entities.h"

CCL_NAMESPACE_BEGIN

class RIBCyclesMesh {
 public:
  RIBCyclesMesh(Scene *scene,
                Instance_Scene_Entity const &inst,
                Instance_Definition_Scene_Entity const *inst_def)
      : _scene(scene), _inst(inst), _inst_def(inst_def)
  {
  }

  ~RIBCyclesMesh()
  {
  }

  void export_geometry();

 protected:
  void initialize();
  void populate_shader_graph(bool initializing = false);

 private:
  Scene *_scene = nullptr;
  Instance_Scene_Entity const &_inst;
  Instance_Definition_Scene_Entity const *_inst_def;
  Mesh *_geom = nullptr;
  std::vector<Object *> _instances;
};

CCL_NAMESPACE_END
#endif  //__EXPORTERS_GEOMETRY_H__
