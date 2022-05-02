
#ifndef __EXPORTERS_GEOMETRY_H__
#define __EXPORTERS_GEOMETRY_H__

#include "scene/scene.h"

#include "app/rib_parser/scene_entities.h"

CCL_NAMESPACE_BEGIN

class RIBCyclesMesh {
 public:
  RIBCyclesMesh(Scene *scene,
                vector<Instance_Scene_Entity> const &inst,
                Instance_Definition_Scene_Entity const *inst_def)
      : _scene(scene), _inst_v(inst), _inst_def(inst_def)
  {
  }

  ~RIBCyclesMesh()
  {
  }

  void export_geometry();

 protected:
  void initialize();
  void initialize_instance(int index);
  void populate(bool &rebuild);
  void populate_normals();
  void populate_primvars();
  void populate_points();
  void populate_topology();
  void populate_shader_graph(bool initializing = false);

 private:
  Scene *_scene = nullptr;
  vector<Instance_Scene_Entity> const &_inst_v;
  Instance_Definition_Scene_Entity const *_inst_def;
  Mesh *_geom = nullptr;
  vector<Object *> _instances;
  ProjectionTransform _geomTransform;
  vector<int3> triangles;

  void compute_triangle_indices(const vector<int> vertices,
                                const vector<int> nvertices,
                                vector<int3> &indices);
};

CCL_NAMESPACE_END
#endif  //__EXPORTERS_GEOMETRY_H__
