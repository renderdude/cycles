#include "scene/mesh.h"
#include "scene/object.h"
#include "scene/shader_graph.h"
#include "scene/shader_nodes.h"

#include "app/rib_parser/exporters/geometry.h"

CCL_NAMESPACE_BEGIN

Mesh *initialize(Scene *scene,
                 Shape_Scene_Entity &shape_inst,
                 Instance_Definition_Scene_Entity *inst_def);
void populate_shader_graph(Shape_Scene_Entity &shape_inst, Mesh *mesh, bool initializing = false);

void export_geometry(Scene *scene,
                     Instance_Scene_Entity &inst,
                     Instance_Definition_Scene_Entity *inst_def)
{
  int shader = 0;
  bool smooth = true;

  // Hack: Find the default_surface shader
  Shader *default_surface;
  for (auto *shader : scene->shaders) {
    if (shader->name == "default_surface") {
      default_surface = shader;
      break;
    }
  }

  for (auto &shape : inst_def->shapes) {
    if (shape.name.find("mesh") != string::npos) {
      ProjectionTransform xform = (*inst.render_from_instance) * (*shape.render_from_object);
      /* create mesh */
      Mesh *mesh = new Mesh();
      scene->geometry.push_back(mesh);

      /* Create object. */
      Object *object = new Object();
      object->set_geometry(mesh);
      object->set_tfm(projection_to_transform(xform));
      scene->objects.push_back(object);

      /* load shader */
      array<Node *> used_shaders = mesh->get_used_shaders();
      used_shaders.push_back_slow(default_surface);
      mesh->set_used_shaders(used_shaders);

      /* process vertex info */
      auto points = shape.parameters.get_point3_array("P");
      auto nverts = shape.parameters.get_int_array("nverts");
      auto verts = shape.parameters.get_int_array("indices");
      array<float3> P_array;
      P_array = points;

      if (mesh->get_subdivision_type() == Mesh::SUBDIVISION_NONE) {
        /* create vertices */

        mesh->set_verts(P_array);

        size_t num_triangles = 0;
        for (size_t i = 0; i < nverts.size(); i++)
          num_triangles += nverts[i] - 2;
        mesh->reserve_mesh(mesh->get_verts().size(), num_triangles);

        /* create triangles */
        int index_offset = 0;

        for (size_t i = 0; i < nverts.size(); i++) {
          for (int j = 0; j < nverts[i] - 2; j++) {
            int v0 = verts[index_offset];
            int v1 = verts[index_offset + j + 1];
            int v2 = verts[index_offset + j + 2];

            assert(v0 < (int)points.size());
            assert(v1 < (int)points.size());
            assert(v2 < (int)points.size());

            // Reverse orientation for cycles
            mesh->add_triangle(v2, v1, v0, shader, smooth);
          }

          index_offset += nverts[i];
        }
      }
      if (mesh->need_attribute(scene, ATTR_STD_GENERATED)) {
        class Attribute *attr = mesh->attributes.add(ATTR_STD_GENERATED);
        memcpy(attr->data_float3(),
               mesh->get_verts().data(),
               sizeof(float3) * mesh->get_verts().size());
      }
    }
    else {
      std::cout << "Found unimplemented geometry type: " << shape.name << std::endl;
    }
  }
}

Mesh *initialize(Scene *scene,
                 Shape_Scene_Entity &shape_inst,
                 Instance_Definition_Scene_Entity *inst_def)
{
}

void populate_shader_graph(Shape_Scene_Entity &shape_inst, Mesh *mesh, bool initializing)
{
}

CCL_NAMESPACE_END
