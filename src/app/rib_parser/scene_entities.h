#ifndef SCENE_ENTITIES_H
#define SCENE_ENTITIES_H

#include <algorithm>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "error.h"
#include "param_dict.h"
#include "util/log.h"
#include "util/projection.h"
#include "util/transform.h"

CCL_NAMESPACE_BEGIN

struct Scene_Entity {
  // Scene_Entity Public Methods
  Scene_Entity() = default;
  Scene_Entity(const std::string &name, Parameter_Dictionary parameters, File_Loc loc)
      : name(name), loc(loc), parameters(parameters)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Scene_Entity name: " << name;
    ss << " parameters: " << parameters.to_string();
    ss << " loc: " << loc.to_string() << " ]";
    return ss.str();
  }

  // Scene_Entity Public Members
  std::string name;
  File_Loc loc;
  Parameter_Dictionary parameters;
};

struct Transformed_Scene_Entity : public Scene_Entity {
  Transformed_Scene_Entity() = default;
  Transformed_Scene_Entity(const std::string &name,
                           Parameter_Dictionary parameters,
                           File_Loc loc,
                           const ProjectionTransform &render_from_object)
      : Scene_Entity(name, parameters, loc), render_from_object(render_from_object)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Transformed_Scene_Entity name: " << name;
    ss << " parameters: " << parameters.to_string();
    ss << " loc: " << loc.to_string();
    //         ss << " render_from_object: " << render_from_object << " ]";
    return ss.str();
  }

  ProjectionTransform render_from_object;
};

// Camera_Scene_Entity Definition
struct Camera_Scene_Entity : public Scene_Entity {
  // Camera_Scene_Entity Public Methods
  Camera_Scene_Entity() = default;
  Camera_Scene_Entity(const std::string &name,
                      Parameter_Dictionary parameters,
                      File_Loc loc,
                      const ProjectionTransform &camera_transform,
                      const std::string &medium)
      : Scene_Entity(name, parameters, loc), camera_transform(camera_transform), medium(medium)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Camera_Scene_Entity name: " << name;
    ss << " parameters: " << parameters.to_string();
    ss << " loc: " << loc.to_string();
    ss << " camera_transform: ";  //<< camera_transform;
    ss << " medium: " << medium << " ]";
    return ss.str();
  }

  ProjectionTransform camera_transform;
  std::string medium;
};

struct Shape_Scene_Entity : public Scene_Entity {
  Shape_Scene_Entity() = default;
  Shape_Scene_Entity(const std::string &name,
                     Parameter_Dictionary parameters,
                     File_Loc loc,
                     ProjectionTransform render_from_object,
                     ProjectionTransform object_from_render,
                     bool reverse_orientation,
                     int material_index,
                     const std::string &material_name,
                     int light_index,
                     const std::string &inside_medium,
                     const std::string &outside_medium)
      : Scene_Entity(name, parameters, loc),
        render_from_object(render_from_object),
        object_from_render(object_from_render),
        reverse_orientation(reverse_orientation),
        material_index(material_index),
        material_name(material_name),
        light_index(light_index),
        inside_medium(inside_medium),
        outside_medium(outside_medium)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Shape_Scene_Entity name: " << name;
    ss << " parameters: " << parameters.to_string();
    ss << " loc: " << loc.to_string();
    ss << " render_from_object: ";  // << render_from_object;
    ss << " object_from_render: ";  // << object_from_render;
    ss << " reverse_orientation: " << reverse_orientation;
    ss << " material_index: " << material_index;
    ss << " material_name: " << material_name;
    ss << " light_index: " << light_index;
    ss << " inside_medium: " << inside_medium;
    ss << " outside_medium: " << outside_medium << "]";
    return ss.str();
  }

  ProjectionTransform render_from_object, object_from_render;
  bool reverse_orientation = false;
  int material_index;  // one of these two...  std::variant?
  std::string material_name;
  int light_index = -1;
  std::string inside_medium, outside_medium;
};

struct Animated_Shape_Scene_Entity : public Transformed_Scene_Entity {
  Animated_Shape_Scene_Entity() = default;
  Animated_Shape_Scene_Entity(const std::string &name,
                              Parameter_Dictionary parameters,
                              File_Loc loc,
                              const ProjectionTransform &render_from_object,
                              const ProjectionTransform *identity,
                              bool reverse_orientation,
                              int material_index,
                              const std::string &material_name,
                              int light_index,
                              const std::string &inside_medium,
                              const std::string &outside_medium)
      : Transformed_Scene_Entity(name, parameters, loc, render_from_object),
        identity(identity),
        reverse_orientation(reverse_orientation),
        material_index(material_index),
        material_name(material_name),
        light_index(light_index),
        inside_medium(inside_medium),
        outside_medium(outside_medium)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Animated_Shape_Scene_Entity name: " << name;
    ss << " parameters: " << parameters.to_string();
    ss << " loc: " << loc.to_string();
    ss << " render_from_object: ";  // << render_from_object;
    ss << " object_from_render: ";  // << object_from_render;
    ss << " reverse_orientation: " << reverse_orientation;
    ss << " material_index: " << material_index;
    ss << " material_name: " << material_name;
    ss << " light_index: " << light_index;
    ss << " inside_medium: " << inside_medium;
    ss << " outside_medium: " << outside_medium << "]";
    return ss.str();
  }

  const ProjectionTransform *identity = nullptr;
  bool reverse_orientation = false;
  int material_index;  // one of these two...  std::variant?
  std::string material_name;
  int light_index = -1;
  std::string inside_medium, outside_medium;
};

struct Instance_Definition_Scene_Entity {
  Instance_Definition_Scene_Entity() = default;
  Instance_Definition_Scene_Entity(const std::string &name, File_Loc loc) : name(name), loc(loc)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Instance_Definition_Scene_Entity name: " << name;
    ss << " loc: " << loc.to_string();
    ss << " shapes: ";           // << shapes;
    ss << " animated_shapes: ";  // << animated_shapes;
    ss << " ]";
    return ss.str();
  }

  std::string name;
  File_Loc loc;
  std::vector<Shape_Scene_Entity> shapes;
  std::vector<Animated_Shape_Scene_Entity> animated_shapes;
};

using Medium_Scene_Entity = Transformed_Scene_Entity;
using Texture_Scene_Entity = Transformed_Scene_Entity;

struct Light_Scene_Entity : public Transformed_Scene_Entity {
  Light_Scene_Entity() = default;
  Light_Scene_Entity(const std::string &name,
                     Parameter_Dictionary parameters,
                     File_Loc loc,
                     const ProjectionTransform &renderFromLight,
                     const std::string &medium)
      : Transformed_Scene_Entity(name, parameters, loc, renderFromLight), medium(medium)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Animated_Shape_Scene_Entity name: " << name;
    ss << " parameters: " << parameters.to_string();
    ss << " loc: " << loc.to_string();
    ss << " render_from_object: ";  // << render_from_object;
    ss << " medium: " << medium;
    return ss.str();
  }

  std::string medium;
};

struct Instance_Scene_Entity {
  Instance_Scene_Entity() = default;
  Instance_Scene_Entity(const std::string &n,
                        File_Loc loc,
                        const std::string &material_name,
                        const ProjectionTransform &render_from_instance_anim)
      : name(n),
        loc(loc),
        material_name(material_name),
        render_from_instance_anim(new ProjectionTransform(render_from_instance_anim))
  {
  }

  Instance_Scene_Entity(const std::string &n,
                        File_Loc loc,
                        const std::string &material_name,
                        const ProjectionTransform *render_from_instance)
      : name(n), loc(loc), material_name(material_name), render_from_instance(render_from_instance)
  {
  }

  // For RenderMan
  Instance_Scene_Entity(const std::string &n,
                        File_Loc loc,
                        const std::string &material_name,
                        Mapped_Parameter_Dictionary params,
                        const ProjectionTransform &render_from_instance_anim)
      : name(n),
        loc(loc),
        material_name(material_name),
        parameters(params),
        render_from_instance_anim(new ProjectionTransform(render_from_instance_anim))
  {
  }

  Instance_Scene_Entity(const std::string &n,
                        File_Loc loc,
                        const std::string &material_name,
                        Mapped_Parameter_Dictionary params,
                        const ProjectionTransform *render_from_instance)
      : name(n),
        loc(loc),
        material_name(material_name),
        parameters(params),
        render_from_instance(render_from_instance)
  {
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "[ Instance_Scene_Entity name: " << name;
    ss << " parameters: ";
    std::for_each(parameters.begin(), parameters.end(), [&](std::pair<std::string, Parameter_Dictionary> parameter)
      {
         ss << "[ " << parameter.first << ": " << parameter.second.to_string() << " ] ";
      });
    ss << " loc: " << loc.to_string();
    ss << " render_from_instance_anim: ";  // << render_from_instance_anim;
    ss << " render_from_instance: ";       // << render_from_instance;
    return ss.str();
  }

  std::string name;
  File_Loc loc;
  std::string material_name;
  Mapped_Parameter_Dictionary parameters;
  ProjectionTransform *render_from_instance_anim = nullptr;
  const ProjectionTransform *render_from_instance = nullptr;
};

// Max_Transforms Definition
constexpr int Max_Transforms = 2;

// Transform_Set Definition
struct Transform_Set {
  // Transform_Set Public Methods
  ProjectionTransform &operator[](int i)
  {
    CHECK_GE(i, 0);
    CHECK_LT(i, Max_Transforms);
    return t[i];
  }
  const ProjectionTransform &operator[](int i) const
  {
    CHECK_GE(i, 0);
    CHECK_LT(i, Max_Transforms);
    return t[i];
  }
  friend Transform_Set inverse(const Transform_Set &ts)
  {
    Transform_Set tInv;
    for (int i = 0; i < Max_Transforms; ++i)
      tInv.t[i] = projection_inverse(ts.t[i]);
    return tInv;
  }
  bool is_animated() const
  {
    for (int i = 0; i < Max_Transforms - 1; ++i)
      if (t[i] != t[i + 1])
        return true;
    return false;
  }

 private:
  ProjectionTransform t[Max_Transforms];
};

CCL_NAMESPACE_END

#endif  // SCENE_ENTITIES_H
