#ifndef RI_API_H
#define RI_API_H

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>

#include "error.h"
#include "intern_cache.h"
#include "param_dict.h"
#include "parsed_parameter.h"
#include "scene_entities.h"

#include "util/projection.h"
#include "util/span.h"
#include "util/transform.h"

CCL_NAMESPACE_BEGIN
using Point3f = float[3];
class Session;

std::vector<std::string> split_string(std::string_view str, char ch);

// Parser Definition
class Ri {
 public:
  Ri(Session *session) : session(session)
  {
  }
  ~Ri();
  // Ri Interface

  void export_to_cycles();
  void export_options(Scene_Entity& filter,
                      Scene_Entity& film,
                      Camera_Scene_Entity& camera,
                      Scene_Entity& sampler);
  void add_light(Light_Scene_Entity light);
  int add_area_light(Scene_Entity light);
  void add_shapes(p_std::span<Shape_Scene_Entity> shape);
  void add_animated_shape(Animated_Shape_Scene_Entity shape);
  void add_instance_definition(Instance_Definition_Scene_Entity instance);
  void add_instance_uses(p_std::span<Instance_Scene_Entity> in);
  void end_of_files();

  std::string get_display_name() const
  {
    return _display_name;
  }

  /*****************************************************************
   ****            Begin RenderMan Definitions                  ****
   *****************************************************************/
  void ArchiveBegin(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void ArchiveEnd(File_Loc loc);
  void AreaLightSource(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Atmosphere(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Attribute(const std::string &target, Parsed_Parameter_Vector params, File_Loc loc);
  void AttributeBegin(File_Loc loc);
  void AttributeEnd(File_Loc loc);
  void Basis(Parsed_Parameter_Vector params, File_Loc loc);
  void Begin(const std::string &name, File_Loc loc);
  void Bound(float bound[6], File_Loc loc);
  void Bxdf(const std::string &bxdf,
            const std::string &name,
            Parsed_Parameter_Vector params,
            File_Loc loc);
  void camera(const std::string &, Parsed_Parameter_Vector params, File_Loc loc);
  void Clipping(float cnear, float cfar, File_Loc loc);
  void ClippingPlane(float x, float y, float z, float nx, float ny, float nz, File_Loc loc);
  void Color(float r, float g, float b, File_Loc loc);
  void Cone(
      float height, float radius, float thetamax, Parsed_Parameter_Vector params, File_Loc loc);
  void ConcatTransform(float transform[16], File_Loc loc);
  void CoordinateSystem(std::string const &name, File_Loc loc);
  void CoordSysTransform(std::string const &name, File_Loc loc);
  void CropWindow(float xmin, float xmax, float ymin, float ymax, File_Loc loc);
  void Curves(const std::string &type,
              std::vector<int> nvertices,
              const std::string &wrap,
              Parsed_Parameter_Vector params,
              File_Loc loc);
  void Cylinder(float radius,
                float zmin,
                float zmax,
                float thetamax,
                Parsed_Parameter_Vector params,
                File_Loc loc);
  void Declare(const std::string &name, const std::string &declaration, File_Loc loc);
  void DepthOfField(float fstop, float focallength, float focaldistance, File_Loc loc);
  void Detail(float bound[6], File_Loc loc);
  void DetailRange(float offlow, float onlow, float onhigh, float offhigh, File_Loc loc);
  void Disk(
      float height, float radius, float thetamax, Parsed_Parameter_Vector params, File_Loc loc);
  void Displacement(const std::string &displace,
                    const std::string &name,
                    Parsed_Parameter_Vector params,
                    File_Loc loc);
  void Display(const std::string &name,
               const std::string &type,
               const std::string &mode,
               Parsed_Parameter_Vector params,
               File_Loc loc);
  void DisplayChannel(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void DisplayFilter(const std::string &name,
                     const std::string &type,
                     Parsed_Parameter_Vector params,
                     File_Loc loc);
  void Else(File_Loc loc);
  void ElseIf(const std::string &condition, File_Loc loc);
  void End(File_Loc loc);
  void Exposure(float gain, float gamma, File_Loc loc);
  void Exterior(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Format(int xresolution, int yresolution, float pixelaspectratio, File_Loc loc);
  void FrameAspectRatio(float frameratio, File_Loc loc);
  void FrameBegin(int number, File_Loc loc);
  void FrameEnd(File_Loc loc);
  void GeneralPolygon(std::vector<int> nvertices, Parsed_Parameter_Vector params, File_Loc loc);
  void GeometricApproximation(const std::string &type, float value, File_Loc loc);
  void Geometry(const std::string &type, Parsed_Parameter_Vector params, File_Loc loc);
  void Hider(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Hyperboloid(Point3f point1,
                   Point3f point2,
                   float thetamax,
                   Parsed_Parameter_Vector params,
                   File_Loc loc);
  void Identity(File_Loc loc);
  void IfBegin(const std::string &condition, File_Loc loc);
  void IfEnd(File_Loc loc);
  void Illuminate(const std::string &light, bool onoff, File_Loc loc);
  void Imager(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Integrator(const std::string &type,
                  const std::string &name,
                  Parsed_Parameter_Vector params,
                  File_Loc loc);
  void Interior(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Light(const std::string &name,
             const std::string &handle,
             Parsed_Parameter_Vector params,
             File_Loc loc);
  void LightSource(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void MakeBrickMap(std::vector<std::string> ptcnames,
                    const std::string &bkmname,
                    Parsed_Parameter_Vector params,
                    File_Loc loc);
  void MakeCubeFaceEnvironment(const std::string &px,
                               const std::string &nx,
                               const std::string &py,
                               const std::string &ny,
                               const std::string &pz,
                               const std::string &nz,
                               const std::string &text,
                               float fov,
                               const std::string &filt,
                               float swidth,
                               float twidth,
                               Parsed_Parameter_Vector params,
                               File_Loc loc);
  void MakeLatLongEnvironment(const std::string &picturename,
                              const std::string &texturename,
                              const std::string &filt,
                              float swidth,
                              float twidth,
                              Parsed_Parameter_Vector params,
                              File_Loc loc);
  void MakeShadow(const std::string &picturename,
                  const std::string &texturename,
                  Parsed_Parameter_Vector params,
                  File_Loc loc);
  void MakeTexture(const std::string &picturename,
                   const std::string &texturename,
                   const std::string &swrap,
                   const std::string &twrap,
                   const std::string &filt,
                   float swidth,
                   float twidth,
                   Parsed_Parameter_Vector params,
                   File_Loc loc);
  void Matte(bool onoff, File_Loc loc);
  void MotionBegin(std::vector<float> times, File_Loc loc);
  void MotionEnd(File_Loc loc);
  void NuPatch(int nu,
               int uorder,
               float uknot[],
               float umin,
               float umax,
               int nv,
               int vorder,
               float vknot[],
               float vmin,
               float vmax,
               Parsed_Parameter_Vector params,
               File_Loc loc);
  void ObjectBegin(const std::string &name, File_Loc loc);
  void ObjectEnd(File_Loc loc);
  void ObjectInstance(const std::string &name, File_Loc loc);
  void Opacity(Parsed_Parameter_Vector params, File_Loc loc);
  void Option(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Orientation(const std::string &orientation, File_Loc loc);
  void Paraboloid(float rmax,
                  float zmin,
                  float zmax,
                  float thetamax,
                  Parsed_Parameter_Vector params,
                  File_Loc loc);
  void Patch(const std::string &type, Parsed_Parameter_Vector params, File_Loc loc);
  void PatchMesh(const std::string &type,
                 int nu,
                 const std::string &uwrap,
                 int nv,
                 const std::string &vwrap,
                 Parsed_Parameter_Vector params,
                 File_Loc loc);
  void Pattern(const std::string &name,
               const std::string &handle,
               Parsed_Parameter_Vector params,
               File_Loc loc);
  void Perspective(float fov, File_Loc loc);
  void PixelFilter(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void PixelSampleImager(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void PixelSamples(float xsamples, float ysamples, File_Loc loc);
  void PixelVariance(float variance, File_Loc loc);
  void Points(int npoints, Parsed_Parameter_Vector params, File_Loc loc);
  void PointsGeneralPolygons(std::vector<int> n_loops,
                             std::vector<int> n_vertices,
                             std::vector<int> vertices,
                             Parsed_Parameter_Vector params,
                             File_Loc loc);
  void PointsPolygons(std::vector<int> n_vertices,
                      std::vector<int> vertices,
                      Parsed_Parameter_Vector params,
                      File_Loc loc);
  void Polygon(int nvertices, Parsed_Parameter_Vector params, File_Loc loc);
  void ProcDelayedReadArchive(Parsed_Parameter_Vector params, File_Loc loc);
  void ProcDynamicLoad(Parsed_Parameter_Vector params, File_Loc loc);
  void Procedural(const std::string &proc_name, Parsed_Parameter_Vector params, File_Loc loc);
  void Procedural2(const std::string &proc_name, Parsed_Parameter_Vector params, File_Loc loc);
  void ProcFree(Parsed_Parameter_Vector params, File_Loc loc);
  void ProcRunProgram(Parsed_Parameter_Vector params, File_Loc loc);
  void Projection(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void Quantize(
      const std::string &type, int one, int min, int max, float ditheramplitude, File_Loc loc);
  void ReadArchive(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void RelativeDetail(float relativedetail, File_Loc loc);
  void Resource(const std::string &handle,
                const std::string &type,
                Parsed_Parameter_Vector params,
                File_Loc loc);
  void ResourceBegin(File_Loc loc);
  void ResourceEnd(File_Loc loc);
  void ReverseOrientation(File_Loc loc);
  void Rotate(float angle, float ax, float ay, float az, File_Loc loc);
  void ScopedCoordinateSystem(const std::string &, File_Loc loc);
  void ScreenWindow(float left, float right, float bottom, float top, File_Loc loc);
  void ShadingInterpolation(const std::string &type, File_Loc loc);
  void ShadingRate(float size, File_Loc loc);
  void Scale(float sx, float sy, float sz, File_Loc loc);
  void Shutter(float opentime, float closetime, File_Loc loc);
  void Sides(int nsides, File_Loc loc);
  void Skew(
      float angle, float dx1, float dy1, float dz1, float dx2, float dy2, float dz2, File_Loc loc);
  void SolidBegin(const std::string &type, File_Loc loc);
  void SolidEnd(File_Loc loc);
  void Sphere(float radius,
              float zmin,
              float zmax,
              float thetamax,
              Parsed_Parameter_Vector params,
              File_Loc loc);
  void SubdivisionMesh(const std::string &scheme,
                       int nfaces,
                       std::vector<int> n_vertices,
                       std::vector<int> vertices,
                       std::vector<std::string> tags,
                       std::vector<int> nargs,
                       std::vector<int> intargs,
                       std::vector<float> floatargs,                       
                       Parsed_Parameter_Vector params,
                       File_Loc loc);
  void Surface(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  void System(const std::string &cmd, File_Loc loc);
  void Texture(const std::string &name,
               const std::string &type,
               const std::string &texname,
               Parsed_Parameter_Vector params,
               File_Loc loc);
  void TextureCoordinates(float s1,
                          float t1,
                          float s2,
                          float t2,
                          float s3,
                          float t3,
                          float s4,
                          float t4,
                          File_Loc loc);
  void Torus(float majorrad,
             float minorrad,
             float phimin,
             float phimax,
             float thetamax,
             Parsed_Parameter_Vector params,
             File_Loc loc);
  void transform(float transform[16], File_Loc loc);
  void TransformBegin(File_Loc loc);
  void TransformEnd(File_Loc loc);
  void Translate(float dx, float dy, float dz, File_Loc loc);
  void TrimCurve(int nloops,
                 int ncurves[],
                 int order[],
                 float knot[],
                 float min[],
                 float max[],
                 int n[],
                 float u[],
                 float v[],
                 float w[],
                 File_Loc loc);
  void WorldBegin(File_Loc loc);
  void WorldEnd(File_Loc loc);

  void add_default_search_paths(std::string filepath);

 protected:
  // Ri Protected Methods
  // Graphics_State Definition
  struct Graphics_State {
    template<typename F> void for_active_transforms(F func)
    {
      for (int i = 0; i < Max_Transforms; ++i)
        if (active_transform_bits & (1 << i))
          ctm[i] = func(ctm[i]);
    }

    // Graphics_State Public Members
    std::string current_inside_medium, current_outside_medium;

    int current_material_index = 0;
    std::string current_material_name;

    std::string area_light_name;
    Parameter_Dictionary area_light_params;
    File_Loc area_light_loc;

    Parsed_Parameter_Vector shape_attributes;
    Parsed_Parameter_Vector light_attributes;
    Parsed_Parameter_Vector material_attributes;
    Parsed_Parameter_Vector medium_attributes;
    Parsed_Parameter_Vector texture_attributes;
    // RenderMan
    std::unordered_map<std::string, Parsed_Parameter_Vector> rib_attributes;

    bool reverse_orientation = false;
    Transform_Set ctm;
    uint32_t active_transform_bits = All_Transforms_Bits;
    float transform_start_time = 0, transform_end_time = 1;
  };

  void error_exit_deferred(File_Loc const *loc, std::string message) const
  {
    errorExit = true;
    error(loc, message);
  }

  ProjectionTransform Render_From_Object(int index) const
  {
    return render_from_world * graphics_state.ctm[index];
  }

  /*
    ProjectionTransform Render_From_Object() const
    {
      return {Render_From_Object(0),
              graphics_state.transform_start_time,
              Render_From_Object(1),
              graphics_state.transform_end_time};
    }
  */

  bool CTM_Is_Animated() const
  {
    return graphics_state.ctm.is_animated();
  }

  struct Active_Instance_Definition {
    Active_Instance_Definition(std::string name, File_Loc loc) : entity(name, loc){};

    std::mutex mutex;
    std::atomic<int> active_imports{1};
    Instance_Definition_Scene_Entity entity;
    Active_Instance_Definition *parent = nullptr;
  };
  Active_Instance_Definition *active_instance_definition = nullptr;

  struct RIB_State {
    RIB_State()
    {
    }

    using Option_Item = std::unordered_map<std::string, Parsed_Parameter *>;
    std::unordered_map<std::string, Option_Item> options;
    float3 world_offset;
  };

  struct OSL_Shader {
    OSL_Shader(std::string bxdf_, std::string name_, Parsed_Parameter_Vector &params_)
        : bxdf(bxdf_), name(name_), params(params_)
    {
    }

    std::string bxdf, name;
    Parsed_Parameter_Vector params;
  };

  void Shape(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc);
  OSL_Shader bxdf_to_osl(std::string bxdf, std::string name, Parsed_Parameter_Vector &params);
  ///@}

  RIB_State _rib_state;
  std::string _shader_id;

  mutable bool errorExit = false;
  Session *session;
  enum class Block_State { Options_Block, World_Block };
  Block_State current_block = Block_State::Options_Block;
  Graphics_State graphics_state;
  static constexpr int Start_Transform_Bits = 1 << 0;
  static constexpr int End_Transform_Bits = 1 << 1;
  static constexpr int All_Transforms_Bits = (1 << Max_Transforms) - 1;
  std::map<std::string, Transform_Set> named_coordinate_systems;
  ProjectionTransform render_from_world;
  std::vector<Graphics_State> pushed_graphics_states;
  std::vector<std::pair<char, File_Loc>> push_stack;  // 'a': attribute, 'o': object
  std::set<std::string> instance_names;
  Intern_Cache<ProjectionTransform, ProjectionTransformHasher> transform_cache;
  std::vector<Parameter_Dictionary> osl_parameters;
  std::map<std::string, std::vector<Parameter_Dictionary>> osl_shader_group;
  // Entity storage
  std::vector<Shape_Scene_Entity> shapes;
  std::map<std::string, vector<Instance_Scene_Entity>> instance_uses;
  std::vector<Animated_Shape_Scene_Entity> animated_shapes;
  std::vector<Instance_Scene_Entity> instances;
  std::map<std::string, Instance_Definition_Scene_Entity *> instance_definitions;
  std::map<std::string, Light_Scene_Entity> _lights;
  std::vector<Scene_Entity> materials;

  std::mutex area_light_mutex;
  std::mutex light_mutex;
  std::mutex shape_mutex, animated_shape_mutex;
  std::mutex instance_definition_mutex, instance_use_mutex;
  std::vector<Scene_Entity> area_lights;
  Scene_Entity film, filter, sampler;
  Camera_Scene_Entity _camera;

  std::string _display_name;
};

#define VERIFY_OPTIONS(func) \
  if (current_block == Block_State::World_Block) { \
    std::stringstream ss; \
    ss << "Options cannot be set inside world block; \"" << func; \
    ss << "\" is not allowed"; \
    error_exit(&loc, ss.str()); \
    return; \
  } \
  else /* swallow trailing semicolon */
#define VERIFY_WORLD(func) \
  if (current_block == Block_State::Options_Block) { \
    std::stringstream ss; \
    ss << "Scene description must be inside world block; \"" << func; \
    ss << "\" is not allowed"; \
    error_exit(&loc, ss.str()); \
    return; \
  } \
  else /* swallow trailing semicolon */

CCL_NAMESPACE_END

#endif  // RI_API_H
