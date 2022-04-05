#include "app/cycles_xml.h"
#include "scene/camera.h"
#include "scene/scene.h"
#include <cmath>
#include <fcntl.h>
#include <sstream>
#include <sys/stat.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <double-conversion/double-conversion.h>
namespace bfs = boost::filesystem;

#include "session/session.h"
#include "util/log.h"

#include "error.h"
#include "ri_api.h"

CCL_NAMESPACE_BEGIN

static bfs::path search_directory;

std::vector<std::string> split_string(std::string_view str, char ch)
{
  std::vector<std::string> strings;

  if (str.empty())
    return strings;

  std::string_view::iterator begin = str.begin();
  while (true) {
    std::string_view::iterator end = begin;
    while (end != str.end() && *end != ch)
      ++end;

    strings.push_back(std::string(begin, end));
    if (end == str.end())
      break;

    begin = end + 1;
  }

  return strings;
}

void Ri::export_to_cycles()
{
  set_options(filter, film, _camera, sampler);
}

inline float radians(float deg)
{
  return (float(M_PI) / 180) * deg;
}

inline float degrees(float rad)
{
  return (180 / float(M_PI)) * rad;
}

void Ri::set_options(Scene_Entity filter,
                        Scene_Entity film,
                        Camera_Scene_Entity camera,
                        Scene_Entity sampler)
{
  // Immediately create filter and film
  VLOG(1) << "Starting to create filter and film";
  // Do something with the filter
  // Filter filt = Filter::create(filter.name, filter.parameters, &filter.loc, alloc);

  // It's a little ugly to poke into the camera's parameters here, but we
  // have this circular dependency that Camera::create() expects a
  // Film, yet now the film needs to know the exposure time from
  // the camera....
  float exposureTime = camera.parameters.get_one_float("shutterclose", 1.f) -
                       camera.parameters.get_one_float("shutteropen", 0.f);
  if (exposureTime <= 0)
    error_exit(&camera.loc,
               "The specified camera shutter times imply that the shutter "
               "does not open.  A black image will result.");

  _display_name = film.parameters.get_one_string("filename", "");

  Camera *cam = session->scene->camera;

  int x_res = film.parameters.get_one_int("xresolution", 1280);
  int y_res = film.parameters.get_one_int("yresolution", 720);
  cam->set_full_width(x_res);
  cam->set_full_height(y_res);
  // cam->set_screen_size(x_res, y_res);

  cam->set_matrix(projection_to_transform(camera.camera_transform.render_from_camera()));
  float near = camera.parameters.get_one_float("nearClip", -1.f);
  if (near > 0)
    cam->set_nearclip(near);
  float far = camera.parameters.get_one_float("farClip", -1.f);
  if (far >= 0)
    cam->set_farclip(far);

  // Set FOV
  float fov = camera.parameters.get_one_float("fov", 45.f);
  cam->set_fov(radians(fov));

  // Set screen window
  auto screen_window = camera.parameters.get_float_array("ScreenWindow");
  cam->set_viewplane_left(screen_window[0]);
  cam->set_viewplane_right(screen_window[1]);
  cam->set_viewplane_bottom(screen_window[2]);
  cam->set_viewplane_top(screen_window[3]);

  cam->need_flags_update = true;
  cam->update(session->scene);
}

void Ri::add_light(Light_Scene_Entity light)
{
  // Medium light_medium = get_medium(light.medium, &light.loc);
  std::lock_guard<std::mutex> lock(light_mutex);

#if 0
  auto create = [this, light, light_medium]() {
    return Light::create(light.name,
                         light.parameters,
                         light.render_from_object.start_transform,
                         get_camera().get_camera_transform(),
                         light_medium,
                         &light.loc,
                         thread_allocators.get());
  };
  light_jobs.push_back(run_async(create));
#endif
}

int Ri::add_area_light(Scene_Entity light)
{
  std::lock_guard<std::mutex> lock(area_light_mutex);
  area_lights.push_back(std::move(light));
  return area_lights.size() - 1;
}

void Ri::add_shapes(p_std::span<Shape_Scene_Entity> s)
{
  std::lock_guard<std::mutex> lock(shape_mutex);
  std::move(std::begin(s), std::end(s), std::back_inserter(shapes));
}

void Ri::add_animated_shape(Animated_Shape_Scene_Entity shape)
{
  std::lock_guard<std::mutex> lock(animated_shape_mutex);
  animated_shapes.push_back(std::move(shape));
}

void Ri::add_instance_definition(Instance_Definition_Scene_Entity instance)
{
  Instance_Definition_Scene_Entity *def = new Instance_Definition_Scene_Entity(
      std::move(instance));

  std::lock_guard<std::mutex> lock(instance_definition_mutex);
  instance_definitions[def->name] = def;
}

void Ri::add_instance_uses(p_std::span<Instance_Scene_Entity> in)
{
  std::lock_guard<std::mutex> lock(instance_use_mutex);
  std::move(std::begin(in), std::end(in), std::back_inserter(instances));
}

Ri::~Ri()
{
}

// RI API Default Implementation
void Ri::ArchiveBegin(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ArchiveBegin is unimplemented" << std::endl;
}

void Ri::ArchiveEnd(File_Loc loc)
{
  std::cout << "ArchiveEnd is unimplemented" << std::endl;
}

void Ri::AreaLightSource(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "AreaLightSource is unimplemented." << std::endl;
}

void Ri::Atmosphere(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Atmosphere is unimplemented" << std::endl;
}

void Ri::Attribute(const std::string &target, Parsed_Parameter_Vector params, File_Loc loc)
{
  for (Parsed_Parameter *p : params) {
    p->may_be_unused = true;
    graphics_state.rib_attributes[target].push_back(p);
  }
}

void Ri::AttributeBegin(File_Loc loc)
{
  VERIFY_WORLD("AttributeBegin");
  osl_parameters.clear();
  pushed_graphics_states.push_back(graphics_state);
  push_stack.push_back(std::make_pair('a', loc));
}

void Ri::AttributeEnd(File_Loc loc)
{
  VERIFY_WORLD("AttributeEnd");
  if (osl_parameters.size() > 0)
    osl_shader_group[_shader_id] = osl_parameters;

  // Issue error on unmatched _AttributeEnd_
  if (pushed_graphics_states.empty()) {
    error(&loc, "Unmatched AttributeEnd encountered. Ignoring it.");
    return;
  }

  // NOTE: must keep the following consistent with code in ObjectEnd
  graphics_state = std::move(pushed_graphics_states.back());
  pushed_graphics_states.pop_back();

  if (push_stack.back().first == 'o') {
    std::stringstream ss;
    ss << "Mismatched nesting: open ObjectBegin from ";
    ss << push_stack.back().second.to_string();
    ss << " at AttributeEnd";
    error_exit_deferred(&loc, ss.str());
  }
  else
    CHECK_EQ(push_stack.back().first, 'a');
  push_stack.pop_back();
}

void Ri::Basis(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Basis is unimplemented" << std::endl;
}

void Ri::Begin(const std::string &name, File_Loc loc)
{
  std::cout << "Begin is unimplemented" << std::endl;
}

void Ri::Bound(float bound[6], File_Loc loc)
{
  std::cout << "Bound is unimplemented" << std::endl;
}

void Ri::Bxdf(const std::string &bxdf,
              const std::string &name,
              Parsed_Parameter_Vector params,
              File_Loc loc)
{
  VERIFY_WORLD("Bxdf");

  OSL_Shader shader = bxdf_to_osl(bxdf, name, params);

  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "string";
  param->name = "bxdf";
  param->add_string(shader.bxdf);
  param->add_string(shader.name);
  shader.params.push_back(param);

  Parameter_Dictionary dict(std::move(shader.params));
  std::string material_id = dict.get_one_string("__materialid", "");
  if (material_id != "") {
    _shader_id = material_id;
    if (osl_shader_group.find(material_id) == osl_shader_group.end())
      osl_parameters.push_back(dict);
    else {
      graphics_state.current_material_name = material_id;
      graphics_state.current_material_index = -1;
    }
  }
}

void Ri::camera(const std::string &, Parsed_Parameter_Vector params, File_Loc loc)
{
  // Remove any class designator from the camera options
  // Specifically, Ri:<something>
  for (auto it = params.begin(); it != params.end(); it++) {
    std::vector<std::string> strings = split_string((*it)->name, ':');
    if (strings.size() > 1)
      // Could extract the size and check we have that many values below
      (*it)->name = strings[1];
  }

  auto options = _rib_state.options["Ri"];

  for (auto s : options) {
    auto *param = s.second;
    bool found = false;
    for (auto it = params.begin(); it != params.end(); it++)
      if ((*it)->name == param->name) {
        found = true;
        break;
      }

    if (!found)
      params.push_back(param);
  }

  Parameter_Dictionary dict(std::move(params));

  VERIFY_OPTIONS("Camera");
  auto items = _rib_state.options.find("trace");
  if (items != _rib_state.options.end()) {
    auto world_origin = items->second.find("worldorigin");
    if (world_origin != items->second.end()) {
      if (world_origin->second->strings[0] == "worldoffset") {
        auto world_offset = items->second["worldoffset"];
        _rib_state.world_offset = -make_float3(
            world_offset->floats[0], world_offset->floats[1], world_offset->floats[2]);
      }
    }
  }

  Transform_Set camera_from_world = graphics_state.ctm;
  Transform_Set world_from_camera = inverse(graphics_state.ctm);
  named_coordinate_systems["camera"] = inverse(camera_from_world);

  // Camera motion
  Camera_Transform camera_transform(world_from_camera[0]);
  render_from_world = camera_transform.render_from_world();

  _camera = Camera_Scene_Entity("perspective",
                                std::move(dict),
                                loc,
                                camera_transform,
                                graphics_state.current_outside_medium);
}

void Ri::Clipping(float cnear, float cfar, File_Loc loc)
{
  std::cout << "Clipping is unimplemented" << std::endl;
}

void Ri::ClippingPlane(float x, float y, float z, float nx, float ny, float nz, File_Loc loc)
{
  std::cout << "ClippingPlane is unimplemented" << std::endl;
}

void Ri::Color(float r, float g, float b, File_Loc loc)
{
  std::cout << "Color is unimplemented" << std::endl;
}

void Ri::ConcatTransform(float transform[16], File_Loc loc)
{
  graphics_state.for_active_transforms([=](auto t) {
    ProjectionTransform projection = *(ProjectionTransform *)&transform[0];
    return t * projection_transpose(projection);
  });
}

void Ri::CoordinateSystem(std::string const &name, File_Loc loc)
{
  named_coordinate_systems[name] = graphics_state.ctm;
}

void Ri::CoordSysTransform(std::string const &name, File_Loc loc)
{
  if (named_coordinate_systems.find(name) != named_coordinate_systems.end())
    graphics_state.ctm = named_coordinate_systems[name];
  else {
    std::stringstream ss;
    ss << "Couldn't find named coordinate system \"" << name << "\"";
    warning(&loc, ss.str());
  }
}

void Ri::Cone(
    float height, float radius, float thetamax, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Cone is unimplemented" << std::endl;
}

void Ri::CropWindow(float xmin, float xmax, float ymin, float ymax, File_Loc loc)
{
  std::cout << "CropWindow is unimplemented" << std::endl;
}

void Ri::Curves(const std::string &type,
                std::vector<int> nvertices,
                const std::string &wrap,
                Parsed_Parameter_Vector params,
                File_Loc loc)
{
  std::cout << "Curves is unimplemented" << std::endl;
}

void Ri::Cylinder(float radius,
                  float zmin,
                  float zmax,
                  float thetamax,
                  Parsed_Parameter_Vector params,
                  File_Loc loc)
{
  VERIFY_WORLD("Shape");

  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "radius";
  param->add_float(radius);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "z_min";
  param->add_float(zmin);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "z_max";
  param->add_float(zmax);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "phi_max";
  param->add_float(thetamax);
  params.push_back(param);

  Shape("cylinder", params, loc);
}

void Ri::Declare(const std::string &name, const std::string &declaration, File_Loc loc)
{
  std::cout << "Declare is unimplemented" << std::endl;
}

void Ri::DepthOfField(float fstop, float focallength, float focaldistance, File_Loc loc)
{
  std::cout << "DepthOfField is unimplemented" << std::endl;
}

void Ri::Detail(float bound[6], File_Loc loc)
{
  std::cout << "Detail is unimplemented" << std::endl;
}

void Ri::DetailRange(float offlow, float onlow, float onhigh, float offhigh, File_Loc loc)
{
  std::cout << "DetailRange is unimplemented" << std::endl;
}

void Ri::Disk(
    float height, float radius, float thetamax, Parsed_Parameter_Vector params, File_Loc loc)
{
  VERIFY_WORLD("Shape");

  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "radius";
  param->add_float(radius);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "height";
  param->add_float(height);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "phimax";
  param->add_float(thetamax);
  params.push_back(param);

  Shape("disk", params, loc);
}

void Ri::Displacement(const std::string &displace,
                      const std::string &name,
                      Parsed_Parameter_Vector params,
                      File_Loc loc)
{
  std::cout << "Displacement is unimplemented" << std::endl;
}

void Ri::Display(const std::string &name,
                 const std::string &type,
                 const std::string &mode,
                 Parsed_Parameter_Vector params,
                 File_Loc loc)
{
  // If the first char of `name' is a '+', then it's an additional
  // channel to render. Ignore it for now.
  if (name[0] == '+') {
    fprintf(stdout, "Ignoring additional display channel, %s\n", name.c_str());
    return;
  }

  Parsed_Parameter_Vector new_params;

  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "string";
  param->name = "filename";
  param->add_string(name);
  new_params.push_back(param);

  auto resolution = _rib_state.options["Ri"]["FormatResolution"]->ints;
  param = new Parsed_Parameter(loc);
  param->type = "integer";
  param->name = "xresolution";
  param->add_int((int)(resolution[0]));
  new_params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "integer";
  param->name = "yresolution";
  param->add_int((int)(resolution[1]));
  new_params.push_back(param);

  Parameter_Dictionary dict(std::move(new_params));
  VERIFY_OPTIONS("Film");
  film = Scene_Entity("rgb", std::move(dict), loc);
}

void Ri::DisplayChannel(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "DisplayChannel " << name << " is unimplemented" << std::endl;
}

void Ri::DisplayFilter(const std::string &name,
                       const std::string &type,
                       Parsed_Parameter_Vector params,
                       File_Loc loc)
{
  std::cout << "DisplayFilter " << name << " is unimplemented" << std::endl;
}

void Ri::Else(File_Loc loc)
{
  std::cout << "Else is unimplemented" << std::endl;
}

void Ri::ElseIf(const std::string &condition, File_Loc loc)
{
  std::cout << "ElseIf is unimplemented" << std::endl;
}

void Ri::End(File_Loc loc)
{
  std::cout << "End is unimplemented" << std::endl;
}

void Ri::Exposure(float gain, float gamma, File_Loc loc)
{
  std::cout << "Exposure is unimplemented" << std::endl;
}

void Ri::Exterior(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Exterior is unimplemented" << std::endl;
}

void Ri::Format(int xresolution, int yresolution, float pixelaspectratio, File_Loc loc)
{
  std::cout << "Format is unimplemented" << std::endl;
}

void Ri::FrameAspectRatio(float frameratio, File_Loc loc)
{
  std::cout << "FrameAspectRatio is unimplemented" << std::endl;
}

void Ri::FrameBegin(int number, File_Loc loc)
{
  std::cout << "FrameBegin is unimplemented" << std::endl;
}

void Ri::FrameEnd(File_Loc loc)
{
  std::cout << "FrameEnd is unimplemented" << std::endl;
}

void Ri::GeneralPolygon(std::vector<int> nvertices, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "GeneralPolygon is unimplemented" << std::endl;
}

void Ri::GeometricApproximation(const std::string &type, float value, File_Loc loc)
{
  std::cout << "GeometricApproximation is unimplemented" << std::endl;
}

void Ri::Geometry(const std::string &type, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Geometry is unimplemented" << std::endl;
}

void Ri::Hider(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Hider is unimplemented" << std::endl;
}

void Ri::Hyperboloid(
    Point3f point1, Point3f point2, float thetamax, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Hyperboloid is unimplemented" << std::endl;
}

void Ri::Identity(File_Loc loc)
{
  graphics_state.for_active_transforms([](auto t) { return projection_identity(); });
}

void Ri::IfBegin(const std::string &condition, File_Loc loc)
{
  std::cout << "IfBegin is unimplemented" << std::endl;
}

void Ri::IfEnd(File_Loc loc)
{
  std::cout << "IfEnd is unimplemented" << std::endl;
}

void Ri::Illuminate(const std::string &light, bool onoff, File_Loc loc)
{
  std::cout << "Illuminate is unimplemented" << std::endl;
}

void Ri::Imager(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Imager is unimplemented" << std::endl;
}

void Ri::Integrator(const std::string &type,
                    const std::string &name,
                    Parsed_Parameter_Vector params,
                    File_Loc loc)
{
  std::cout << "Integrator is unimplemented" << std::endl;
}

void Ri::Interior(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Interior is unimplemented" << std::endl;
}

void Ri::Light(const std::string &name,
               const std::string &handle,
               Parsed_Parameter_Vector params,
               File_Loc loc)
{
  VERIFY_WORLD("Light");
  Parsed_Parameter *exposure, *intensity, *light_color;
  Parsed_Parameter_Vector shape_params;

  bool double_sided = false;
  for (auto it = graphics_state.shape_attributes.begin();
       it != graphics_state.shape_attributes.end();
       it++)
    if ((*it)->name == "sides") {
      double_sided = (*it)->floats[0] == 2;
      break;
    }

  Parsed_Parameter *param;
  // Lights are single-sided by default
  if (double_sided) {
    param = new Parsed_Parameter(loc);
    param->type = "bool";
    param->name = "twosided";
    param->may_be_unused = true;
    param->add_bool(true);
    params.push_back(param);
  }

  // Need to inform the shape creation routines that this is a
  // light.
  param = new Parsed_Parameter(loc);
  param->type = "bool";
  param->name = "is_light";
  param->may_be_unused = true;
  param->add_bool(true);
  shape_params.push_back(param);

  graphics_state.area_light_name = "diffuse";
  graphics_state.area_light_params = Parameter_Dictionary(std::move(params),
                                                          graphics_state.light_attributes);
  graphics_state.area_light_loc = loc;

  // RenderMan and Cycles have different notions about what side is front
  // for a Disk
  if (name == "PxrDiskLight")
    graphics_state.reverse_orientation = !graphics_state.reverse_orientation;

  if (name == "PxrSphereLight")
    Sphere(0.5f, -0.5f, 0.5f, 360.f, shape_params, loc);
  else if (name == "PxrCylinderLight")
    Cylinder(0.5f, -0.5f, 0.5f, 360.f, shape_params, loc);
  else if (name == "PxrDiskLight")
    Disk(0.f, 0.5f, 360.f, shape_params, loc);
  else if (name == "PxrRectLight") {
    std::vector<int> n_vertices = {4};
    std::vector<int> vertices = {0, 2, 3, 1};
    std::vector<float> points = {-0.5, -0.5, 0, 0.5, -0.5, 0, -0.5, 0.5, 0, 0.5, 0.5, 0};

    param = new Parsed_Parameter(loc);
    param->storage = Container_Type::Vertex;
    param->type = "point3";
    param->name = "P";
    for (auto p : points)
      param->add_float(p);
    shape_params.push_back(param);
    PointsPolygons(n_vertices, vertices, shape_params, loc);
  }
}

void Ri::LightSource(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "LightSource is unimplemented." << std::endl;
}

void Ri::MakeBrickMap(std::vector<std::string> ptcnames,
                      const std::string &bkmname,
                      Parsed_Parameter_Vector params,
                      File_Loc loc)
{
  std::cout << "MakeBrickMap is unimplemented" << std::endl;
}

void Ri::MakeCubeFaceEnvironment(const std::string &px,
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
                                 File_Loc loc)
{
  std::cout << "MakeCubeFaceEnvironment is unimplemented" << std::endl;
}

void Ri::MakeLatLongEnvironment(const std::string &picturename,
                                const std::string &texturename,
                                const std::string &filt,
                                float swidth,
                                float twidth,
                                Parsed_Parameter_Vector params,
                                File_Loc loc)
{
  std::cout << "MakeLatLongEnvironment is unimplemented" << std::endl;
}

void Ri::MakeShadow(const std::string &picturename,
                    const std::string &texturename,
                    Parsed_Parameter_Vector params,
                    File_Loc loc)
{
  std::cout << "MakeShadow is unimplemented" << std::endl;
}

void Ri::MakeTexture(const std::string &picturename,
                     const std::string &texturename,
                     const std::string &swrap,
                     const std::string &twrap,
                     const std::string &filt,
                     float swidth,
                     float twidth,
                     Parsed_Parameter_Vector params,
                     File_Loc loc)
{
  std::cout << "MakeTexture is unimplemented" << std::endl;
}

void Ri::Matte(bool onoff, File_Loc loc)
{
  std::cout << "Matte is unimplemented" << std::endl;
}

void Ri::MotionBegin(std::vector<float> times, File_Loc loc)
{
  std::cout << "MotionBegin is unimplemented" << std::endl;
}

void Ri::MotionEnd(File_Loc loc)
{
  std::cout << "MotionEnd is unimplemented" << std::endl;
}

void Ri::NuPatch(int nu,
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
                 File_Loc loc)
{
  std::cout << "NuPatch is unimplemented" << std::endl;
}

void Ri::ObjectBegin(const std::string &name, File_Loc loc)
{
  VERIFY_WORLD("ObjectBegin");
  pushed_graphics_states.push_back(graphics_state);

  push_stack.push_back(std::make_pair('o', loc));

  if (active_instance_definition) {
    error_exit_deferred(&loc, "ObjectBegin called inside of instance definition");
    return;
  }

  if (instance_names.find(name) != instance_names.end()) {
    std::stringstream ss;
    ss << name << ": trying to redefine an object instance";
    error_exit_deferred(&loc, ss.str());
    return;
  }
  instance_names.insert(name);
  active_instance_definition = new Active_Instance_Definition(name, loc);
}

void Ri::ObjectEnd(File_Loc loc)
{
  VERIFY_WORLD("ObjectEnd");
  if (!active_instance_definition) {
    error_exit_deferred(&loc, "ObjectEnd called outside of instance definition");
    return;
  }
  if (active_instance_definition->parent) {
    error_exit_deferred(&loc, "ObjectEnd called inside Import for instance definition");
    return;
  }

  // NOTE: Must keep the following consistent with AttributeEnd
  graphics_state = std::move(pushed_graphics_states.back());
  pushed_graphics_states.pop_back();

  if (push_stack.back().first == 'a') {
    std::stringstream ss;
    ss << "Mismatched nesting: open AttributeBegin from ";
    ss << push_stack.back().second.to_string();
    ss << " at ObjectEnd";
    error_exit_deferred(&loc, ss.str());
  }
  else
    CHECK_EQ(push_stack.back().first, 'o');
  push_stack.pop_back();

  if (--active_instance_definition->active_imports == 0) {
    add_instance_definition(std::move(active_instance_definition->entity));
    delete active_instance_definition;
  }

  active_instance_definition = nullptr;
}

void Ri::ObjectInstance(const std::string &name, File_Loc loc)
{
  VERIFY_WORLD("ObjectInstance");

  Mapped_Parameter_Dictionary dict;
  for (auto &x : graphics_state.rib_attributes) {
    Parameter_Dictionary d(x.second);

    dict.emplace(x.first, d);
  }

  if (active_instance_definition) {
    error_exit_deferred(&loc, "ObjectInstance can't be called inside instance definition");
    return;
  }

  ProjectionTransform world_from_render = projection_inverse(render_from_world);

#if 0
      if ( CTM_Is_Animated() )
      {
         Animated_Transform animated_render_from_instance(
             Render_From_Object( 0 ) * world_from_render,
             graphics_state.transform_start_time,
             Render_From_Object( 1 ) * world_from_render,
             graphics_state.transform_end_time );

         // For very small changes, animated_render_from_instance may have both
         // xforms equal even if CTMIsAnimated() has returned true. Fall
         // through to create a regular non-animated instance in that case.
         if ( animated_render_from_instance.is_animated() )
         {
            instance_uses.push_back( Instance_Scene_Entity(
                name, loc, graphics_state.current_material_name,
                animated_render_from_instance ) );
            return;
         }
      }

      const class Transform* render_from_instance = scene->transform_cache.lookup(
          Render_From_Object( 0 ) * world_from_render );
#endif

  const ProjectionTransform *render_from_instance = transform_cache.lookup(Render_From_Object(0) *
                                                                           world_from_render);
  instance_uses.push_back(Instance_Scene_Entity(name,
                                                loc,
                                                graphics_state.current_material_name,
                                                std::move(dict) /* RenderMan*/,
                                                render_from_instance));
}

void Ri::Option(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  auto items = _rib_state.options.find(name);
  if (items == _rib_state.options.end()) {
    _rib_state.options[name] = {};
    items = _rib_state.options.find(name);
  }

  for (auto *p : params) {
    // FIX_ME: If the name is already in the map, this will
    // result in a memory leak
    items->second[p->name] = p;
  }
}

void Ri::Opacity(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Opacity is unimplemented" << std::endl;
}

void Ri::Orientation(const std::string &orientation, File_Loc loc)
{
  std::cout << "Orientation is unimplemented" << std::endl;
}

void Ri::Paraboloid(float rmax,
                    float zmin,
                    float zmax,
                    float thetamax,
                    Parsed_Parameter_Vector params,
                    File_Loc loc)
{
  std::cout << "Paraboloid is unimplemented" << std::endl;
}

void Ri::Patch(const std::string &type, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Patch is unimplemented" << std::endl;
}

void Ri::PatchMesh(const std::string &type,
                   int nu,
                   const std::string &uwrap,
                   int nv,
                   const std::string &vwrap,
                   Parsed_Parameter_Vector params,
                   File_Loc loc)
{
  std::cout << "PatchMesh is unimplemented" << std::endl;
}

void Ri::Pattern(const std::string &name,
                 const std::string &handle,
                 Parsed_Parameter_Vector params,
                 File_Loc loc)
{
  VERIFY_WORLD("Pattern");

  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "string";
  param->name = "pattern";
  param->add_string(name);
  param->add_string(handle);
  params.push_back(param);

  Parameter_Dictionary dict(std::move(params));
  osl_parameters.push_back(dict);
}

void Ri::Perspective(float fov, File_Loc loc)
{
  std::cout << "Perspective is unimplemented" << std::endl;
}

void Ri::PixelFilter(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "PixelFilter is unimplemented." << std::endl;
}

void Ri::PixelSampleImager(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "PixelSampleImager is unimplemented" << std::endl;
}

void Ri::PixelSamples(float xsamples, float ysamples, File_Loc loc)
{
  std::cout << "PixelSamples is unimplemented" << std::endl;
}

void Ri::PixelVariance(float variance, File_Loc loc)
{
  std::cout << "PixelVariance is unimplemented" << std::endl;
}

void Ri::Points(int npoints, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Points is unimplemented" << std::endl;
}

void Ri::PointsGeneralPolygons(std::vector<int> n_loops,
                               std::vector<int> n_vertices,
                               std::vector<int> vertices,
                               Parsed_Parameter_Vector params,
                               File_Loc loc)
{
  bool single_only = true;

  // Only single loops for now
  for (int i = 0; single_only && i < n_loops.size(); i++)
    single_only = ((int)n_loops[i] == 1);

  if (single_only)
    PointsPolygons(n_vertices, vertices, params, loc);
}

void Ri::PointsPolygons(std::vector<int> n_vertices,
                        std::vector<int> vertices,
                        Parsed_Parameter_Vector params,
                        File_Loc loc)
{
  bool needs_tessellation = false;
  int base_vert_count = n_vertices[0];
  for (auto &i : n_vertices)
    if (i > 4) {
      needs_tessellation = true;
      base_vert_count = i;
      break;
    }

  if (needs_tessellation)
    fprintf(stdout,
            "The polygons have greater than 4 vertices per poly (%d found). "
            "Ignoring the shape until tesselation is implemented.",
            base_vert_count);
  else {
    Parsed_Parameter *param = new Parsed_Parameter(loc);
    param->type = "integer";
    param->name = "indices";
    for (int i = 0; i < vertices.size(); ++i)
      param->add_int(vertices[i]);
    params.push_back(param);

    param = new Parsed_Parameter(loc);
    param->type = "integer";
    param->name = "nverts";
    for (int i = 0; i < n_vertices.size(); ++i)
      param->add_int(n_vertices[i]);
    params.push_back(param);

    // Fix attributes whose storage class couldn't be determined at parsing
    // time
    for (auto &p : params) {
      if ((p->type == "point3" || p->type == "normal" || p->type == "rgb") &&
          p->storage == Container_Type::Constant && p->floats.size() > 3) {
        int num_vals = p->floats.size() / 3;
        if (num_vals == n_vertices.size())
          p->storage = Container_Type::Uniform;
        else if (num_vals == vertices.size())
          p->storage = Container_Type::Vertex;
      }
    }

    Shape("mesh", params, loc);
  }
}

void Ri::Polygon(int nvertices, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Polygon is unimplemented" << std::endl;
}

void Ri::ProcDelayedReadArchive(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcDelayedReadArchive is unimplemented" << std::endl;
}

void Ri::ProcDynamicLoad(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcDynamicLoad is unimplemented" << std::endl;
}

void Ri::Procedural(const std::string &proc_name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Procedural is unimplemented" << std::endl;
}

void Ri::Procedural2(const std::string &proc_name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Procedural2 is unimplemented" << std::endl;
}

void Ri::ProcFree(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcFree is unimplemented" << std::endl;
}

void Ri::ProcRunProgram(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcRunProgram is unimplemented" << std::endl;
}

void Ri::Projection(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  auto items = _rib_state.options.find("Ri");
  if (items == _rib_state.options.end()) {
    _rib_state.options["Ri"] = {};
    items = _rib_state.options.find(name);
  }

  for (auto *p : params) {
    // FIX_ME: If the name already is in the map, this will
    // result in a memory leak
    items->second[p->name] = p;
  }
}

void Ri::Quantize(
    const std::string &type, int one, int min, int max, float ditheramplitude, File_Loc loc)
{
  std::cout << "Quantize is unimplemented" << std::endl;
}

void Ri::ReadArchive(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ReadArchive is unimplemented" << std::endl;
}

void Ri::RelativeDetail(float relativedetail, File_Loc loc)
{
  std::cout << "RelativeDetail is unimplemented" << std::endl;
}

void Ri::Resource(const std::string &handle,
                  const std::string &type,
                  Parsed_Parameter_Vector params,
                  File_Loc loc)
{
  std::cout << "Resource is unimplemented" << std::endl;
}

void Ri::ResourceBegin(File_Loc loc)
{
  std::cout << "ResourceBegin is unimplemented" << std::endl;
}

void Ri::ResourceEnd(File_Loc loc)
{
  std::cout << "ResourceEnd is unimplemented" << std::endl;
}

void Ri::ReverseOrientation(File_Loc loc)
{
  VERIFY_WORLD("ReverseOrientation");
  graphics_state.reverse_orientation = !graphics_state.reverse_orientation;
}

void Ri::Rotate(float angle, float ax, float ay, float az, File_Loc loc)
{
  graphics_state.for_active_transforms(
      [=](auto t) { return t * transform_rotate(DEG2RADF(angle), make_float3(ax, ay, az)); });
}

void Ri::Scale(float sx, float sy, float sz, File_Loc loc)
{
  graphics_state.for_active_transforms(
      [=](auto t) { return t * transform_scale(make_float3(sx, sy, sz)); });
}

void Ri::ScopedCoordinateSystem(const std::string &, File_Loc loc)
{
  std::cout << "ScopedCoordinateSystem is unimplemented" << std::endl;
}

void Ri::ScreenWindow(float left, float right, float bottom, float top, File_Loc loc)
{
  std::cout << "ScreenWindow is unimplemented" << std::endl;
}

void Ri::ShadingInterpolation(const std::string &type, File_Loc loc)
{
  std::cout << "ShadingInterpolation is unimplemented" << std::endl;
}

void Ri::ShadingRate(float size, File_Loc loc)
{
  std::cout << "ShadingRate is unimplemented" << std::endl;
}

void Ri::Shutter(float opentime, float closetime, File_Loc loc)
{
  std::cout << "Shutter is unimplemented" << std::endl;
}

void Ri::Sides(int nsides, File_Loc loc)
{
  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "integer";
  param->name = "sides";
  param->add_float(nsides);
  graphics_state.shape_attributes.push_back(param);
}

void Ri::Skew(
    float angle, float dx1, float dy1, float dz1, float dx2, float dy2, float dz2, File_Loc loc)
{
  std::cout << "Skew is unimplemented" << std::endl;
}

void Ri::SolidBegin(const std::string &type, File_Loc loc)
{
  std::cout << "SolidBegin is unimplemented" << std::endl;
}

void Ri::SolidEnd(File_Loc loc)
{
  std::cout << "SolidEnd is unimplemented" << std::endl;
}

void Ri::Sphere(float radius,
                float zmin,
                float zmax,
                float thetamax,
                Parsed_Parameter_Vector params,
                File_Loc loc)
{
  VERIFY_WORLD("Shape");

  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "radius";
  param->add_float(radius);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "z_min";
  param->add_float(zmin);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "z_max";
  param->add_float(zmax);
  params.push_back(param);

  param = new Parsed_Parameter(loc);
  param->type = "float";
  param->name = "phi_max";
  param->add_float(thetamax);
  params.push_back(param);

  Shape("sphere", params, loc);
}

void Ri::SubdivisionMesh(const std::string &scheme,
                         int nfaces,
                         Parsed_Parameter_Vector params,
                         File_Loc loc)
{
  std::cout << "SubdivisionMesh is unimplemented" << std::endl;
}

void Ri::Surface(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Surface is unimplemented" << std::endl;
}

void Ri::System(const std::string &cmd, File_Loc loc)
{
  std::cout << "System is unimplemented" << std::endl;
}

void Ri::Texture(const std::string &name,
                 const std::string &type,
                 const std::string &texname,
                 Parsed_Parameter_Vector params,
                 File_Loc loc)
{
  std::cout << "Texture is unimplemented." << std::endl;
}

void Ri::TextureCoordinates(
    float s1, float t1, float s2, float t2, float s3, float t3, float s4, float t4, File_Loc loc)
{
  std::cout << "TextureCoordinates is unimplemented" << std::endl;
}

void Ri::Torus(float majorrad,
               float minorrad,
               float phimin,
               float phimax,
               float thetamax,
               Parsed_Parameter_Vector params,
               File_Loc loc)
{
  std::cout << "Torus is unimplemented" << std::endl;
}

void Ri::transform(float transform[16], File_Loc loc)
{
  graphics_state.for_active_transforms([=](auto t) {
    // Stomp the current transform
    ProjectionTransform projection = *(ProjectionTransform *)&transform[0];
    return projection_transpose(projection);
  });
}

void Ri::TransformBegin(File_Loc loc)
{
  std::cout << "TransformBegin is unimplemented" << std::endl;
}

void Ri::TransformEnd(File_Loc loc)
{
  std::cout << "TransformEnd is unimplemented" << std::endl;
}

void Ri::Translate(float dx, float dy, float dz, File_Loc loc)
{
  graphics_state.for_active_transforms(
      [=](auto t) { return t * transform_translate(make_float3(dx, dy, dz)); });
}

void Ri::TrimCurve(int nloops,
                   int ncurves[],
                   int order[],
                   float knot[],
                   float min[],
                   float max[],
                   int n[],
                   float u[],
                   float v[],
                   float w[],
                   File_Loc loc)
{
  std::cout << "TrimCurve is unimplemented" << std::endl;
}

void Ri::WorldBegin(File_Loc loc)
{
  VERIFY_OPTIONS("WorldBegin");
  // Reset graphics state for _WorldBegin_
  current_block = Block_State::World_Block;
  for (int i = 0; i < Max_Transforms; ++i)
    graphics_state.ctm[i] = projection_identity();
  named_coordinate_systems["world"] = graphics_state.ctm;

  Parsed_Parameter_Vector params;
  Parsed_Parameter *param;

  auto options = _rib_state.options["Ri"];
  auto *opt_param = options["PixelFilterName"];
  if (opt_param != nullptr) {
    std::string name = opt_param->strings[0];
    float xradius = 2, yradius = 2;

    auto *opt_width = options["PixelFilterWidth"];
    if (opt_width != nullptr) {
      xradius = opt_width->floats[0];
      yradius = opt_width->floats[1];
    }

    param = new Parsed_Parameter(loc);
    param->type = "float";
    param->name = "xradius";
    param->add_float(xradius);
    params.push_back(param);

    param = new Parsed_Parameter(loc);
    param->type = "float";
    param->name = "yradius";
    param->add_float(yradius);
    params.push_back(param);

    Parameter_Dictionary dict(std::move(params));
    filter = Scene_Entity(name, std::move(dict), loc);
  }

  opt_param = options["PixelVariance"];
  if (opt_param != nullptr) {
    float pixel_var = opt_param->floats[0];

    param = new Parsed_Parameter(loc);
    param->type = "float";
    param->name = "pixel_variance";
    param->add_float(pixel_var);
    sampler.parameters.push_back(param);
  }

  options = _rib_state.options["searchpath"];
  opt_param = options["shader"];
  std::string paths = opt_param->strings[0] + ":";

  options = _rib_state.options["hider"];
  opt_param = options["minsamples"];
  if (opt_param != nullptr) {
    int min_samples = opt_param->ints[0];
    opt_param = options["maxsamples"];
    int max_samples = opt_param->ints[0];

    param = new Parsed_Parameter(loc);
    param->type = "integer";
    param->name = "min_samples";
    param->add_int(min_samples);
    sampler.parameters.push_back(param);

    param = new Parsed_Parameter(loc);
    param->type = "integer";
    param->name = "max_samples";
    param->add_int(max_samples);
    sampler.parameters.push_back(param);
  }
}

void Ri::WorldEnd(File_Loc loc)
{
  std::cout << "WorldEnd is unimplemented" << std::endl;
}

void Ri::end_of_files()
{
  // Do end of parsing operations
}

void Ri::add_default_search_paths(std::string filepath)
{
  Parsed_Parameter_Vector params;
  Parsed_Parameter *param;

  param = new Parsed_Parameter(File_Loc());
  param->type = "string";
  param->name = "shader_default";
  param->add_string(filepath);

  params.push_back(param);

  Option("searchpath", params, File_Loc());
}

void Ri::Shape(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  VERIFY_WORLD("Shape");

  Parameter_Dictionary dict(std::move(params), graphics_state.shape_attributes);

  int area_light_index = -1;
  if (!graphics_state.area_light_name.empty()) {
    area_light_index = add_area_light(Scene_Entity(graphics_state.area_light_name,
                                                   graphics_state.area_light_params,
                                                   graphics_state.area_light_loc));
  }
#if 0
  if (CTM_Is_Animated()) {
    Animated_Transform renderFromShape = Render_From_Object();
    const class Transform *identity = scene->transform_cache.lookup(papillon::Transform());

    Animated_Shape_Scene_Entity entity({name,
                                        std::move(dict),
                                        loc,
                                        renderFromShape,
                                        identity,
                                        graphics_state.reverse_orientation,
                                        graphics_state.current_material_index,
                                        graphics_state.current_material_name,
                                        area_light_index,
                                        graphics_state.current_inside_medium,
                                        graphics_state.current_outside_medium});

    if (active_instance_definition)
      active_instance_definition->entity.animated_shapes.push_back(std::move(entity));
    else
      scene->add_animated_shape(std::move(entity));
  }
  else {
#endif
  ProjectionTransform const *render_from_object = transform_cache.lookup(Render_From_Object(0));
  ProjectionTransform const *object_from_render = transform_cache.lookup(
      projection_inverse(*render_from_object));

  Shape_Scene_Entity entity({name,
                             std::move(dict),
                             loc,
                             render_from_object,
                             object_from_render,
                             graphics_state.reverse_orientation,
                             graphics_state.current_material_index,
                             graphics_state.current_material_name,
                             area_light_index,
                             graphics_state.current_inside_medium,
                             graphics_state.current_outside_medium});
  if (active_instance_definition)
    active_instance_definition->entity.shapes.push_back(std::move(entity));
  else
    shapes.push_back(std::move(entity));
}

Ri::OSL_Shader Ri::bxdf_to_osl(std::string bxdf, std::string name, Parsed_Parameter_Vector &params)
{
  std::map<std::string, Parsed_Parameter *> param_table;
  for (auto *p : params)
    param_table[p->name] = p;

  bool coated_diffuse = false;

  Parsed_Parameter *spec_mode = param_table["specularFresnelMode"];
  // Check if we're in the BxDF definition or in the reference (which only
  // contains __materialid)
  if (spec_mode) {
    if (spec_mode->ints[0] == 0)  // Artistic mode
    {
      Parsed_Parameter *param = param_table["specularFaceColor"];
      if (!param->floats_are(0))
        coated_diffuse = true;
    }
    else  // Physical mode
    {
      Parsed_Parameter *param = param_table["specularEdgeColor"];
      if (!param->floats_are(0))
        coated_diffuse = true;
    }

    if (!coated_diffuse) {
      Parsed_Parameter *cc_mode = param_table["clearcoatFresnelMode"];
      if (cc_mode->ints[0] == 0)  // Artistic mode
      {
        Parsed_Parameter *param = param_table["clearcoatFaceColor"];
        if (!param->floats_are(0))
          coated_diffuse = true;
      }
      else  // Physical mode
      {
        Parsed_Parameter *param = param_table["clearcoatEdgeColor"];
        if (!param->floats_are(0))
          coated_diffuse = true;
      }
    }

    if (coated_diffuse) {
      std::vector<std::string> names{"diffuseGain",
                                     "diffuseColor",
                                     "diffuseRoughness",
                                     "diffuseExponent",
                                     "diffuseBumpNormal",
                                     "diffuseDoubleSided",
                                     "diffuseBackUseDiffuseColor",
                                     "diffuseBackColor",
                                     "diffuseTransmitGain",
                                     "diffuseTransmitColor",
                                     "specularFresnelMode",
                                     "specularFaceColor",
                                     "specularEdgeColor",
                                     "specularFresnelShape",
                                     "specularIor",
                                     "specularExtinctionCoeff",
                                     "specularRoughness",
                                     "specularModelType",
                                     "specularAnisotropy",
                                     "specularAnisotropyDirection",
                                     "specularBumpNormal",
                                     "specularDoubleSided",
                                     "roughSpecularFresnelMode",
                                     "roughSpecularFaceColor",
                                     "roughSpecularEdgeColor",
                                     "roughSpecularFresnelShape",
                                     "roughSpecularIor",
                                     "roughSpecularExtinctionCoeff",
                                     "roughSpecularRoughness",
                                     "roughSpecularModelType",
                                     "roughSpecularAnisotropy",
                                     "roughSpecularAnisotropyDirection",
                                     "roughSpecularBumpNormal",
                                     "roughSpecularDoubleSided",
                                     "clearcoatFresnelMode",
                                     "clearcoatFaceColor",
                                     "clearcoatEdgeColor",
                                     "clearcoatFresnelShape",
                                     "clearcoatIor",
                                     "clearcoatExtinctionCoeff",
                                     "clearcoatThickness",
                                     "clearcoatAbsorptionTint",
                                     "clearcoatRoughness",
                                     "clearcoatModelType",
                                     "clearcoatAnisotropy",
                                     "clearcoatAnisotropyDirection",
                                     "clearcoatBumpNormal",
                                     "clearcoatDoubleSided",
                                     "specularEnergyCompensation",
                                     "clearcoatEnergyCompensation",
                                     "__materialid"};
      Parsed_Parameter_Vector cd_params;
      for (auto name : names)
        cd_params.push_back(param_table[name]);
      return OSL_Shader("coated_diffuse", name, cd_params);
    }
    else
      return OSL_Shader(bxdf, name, params);
  }
  else
    return OSL_Shader(bxdf, name, params);
}

CCL_NAMESPACE_END
