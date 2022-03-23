#include "app/cycles_xml.h"
#include "scene/camera.h"
#include "scene/scene.h"
#include "util/projection.h"
#include "util/transform.h"
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

#include "util/log.h"

#include "error.h"
#include "parser.h"

CCL_NAMESPACE_BEGIN

static bfs::path search_directory;

void set_search_directory(std::string filename)
{
  bfs::path path(filename);
  if (!bfs::is_directory(path))
    path = path.parent_path();
  search_directory = path;
}

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

Parser::~Parser()
{
}

// RI API Default Implementation
void Parser::ArchiveBegin(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ArchiveBegin is unimplemented" << std::endl;
}

void Parser::ArchiveEnd(File_Loc loc)
{
  std::cout << "ArchiveEnd is unimplemented" << std::endl;
}

void Parser::AreaLightSource(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "AreaLightSource is unimplemented." << std::endl;
}

void Parser::Atmosphere(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Atmosphere is unimplemented" << std::endl;
}

void Parser::Attribute(const std::string &target, Parsed_Parameter_Vector params, File_Loc loc)
{
  for (Parsed_Parameter *p : params) {
    p->may_be_unused = true;
    graphics_state.rib_attributes[target].push_back(p);
  }
}

void Parser::AttributeBegin(File_Loc loc)
{
  VERIFY_WORLD("AttributeBegin");
  osl_parameters.clear();
  pushed_graphics_states.push_back(graphics_state);
  push_stack.push_back(std::make_pair('a', loc));
}

void Parser::AttributeEnd(File_Loc loc)
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

void Parser::Basis(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Basis is unimplemented" << std::endl;
}

void Parser::Begin(const std::string &name, File_Loc loc)
{
  std::cout << "Begin is unimplemented" << std::endl;
}

void Parser::Bound(float bound[6], File_Loc loc)
{
  std::cout << "Bound is unimplemented" << std::endl;
}

void Parser::Bxdf(const std::string &bxdf,
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

void Parser::camera(const std::string &, Parsed_Parameter_Vector params, File_Loc loc)
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
  Parsed_Parameter_Vector new_params;

  std::vector<std::string> camera_options = {"fov", "focalLength", "ScreenWindow"};
  for (auto s : camera_options) {
    auto *param = options[s];
    for (auto it = params.begin(); it != params.end(); it++)
      if ((*it)->name == s) {
        param = *it;
        break;
      }

    new_params.push_back(param);
  }

  Parameter_Dictionary dict(std::move(new_params));

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

#if 0
  // Camera motion
  Camera_Transform camera_transform(Animated_Transform(world_from_camera[0],
                                                       graphics_state.transform_start_time,
                                                       world_from_camera[1],
                                                       graphics_state.transform_end_time));
  render_from_world = camera_transform.render_from_world();
#endif

  _camera = Camera_Scene_Entity("perspective",
                                std::move(dict),
                                loc,
                                graphics_state.ctm,
                                graphics_state.current_outside_medium);
}

void Parser::Clipping(float cnear, float cfar, File_Loc loc)
{
  std::cout << "Clipping is unimplemented" << std::endl;
}

void Parser::ClippingPlane(float x, float y, float z, float nx, float ny, float nz, File_Loc loc)
{
  std::cout << "ClippingPlane is unimplemented" << std::endl;
}

void Parser::Color(float r, float g, float b, File_Loc loc)
{
  std::cout << "Color is unimplemented" << std::endl;
}

void Parser::ConcatTransform(float transform[16], File_Loc loc)
{
  ProjectionTransform projection = *(ProjectionTransform *)&transform[0];
  graphics_state.ctm = graphics_state.ctm *
                       projection_to_transform(projection_transpose(projection));
}

void Parser::CoordinateSystem(std::string const &name, File_Loc loc)
{
  named_coordinate_systems[name] = graphics_state.ctm;
}

void Parser::CoordSysTransform(std::string const &name, File_Loc loc)
{
  if (named_coordinate_systems.find(name) != named_coordinate_systems.end())
    graphics_state.ctm = named_coordinate_systems[name];
  else {
    std::stringstream ss;
    ss << "Couldn't find named coordinate system \"" << name << "\"";
    warning(&loc, ss.str());
  }
}

void Parser::Cone(
    float height, float radius, float thetamax, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Cone is unimplemented" << std::endl;
}

void Parser::CropWindow(float xmin, float xmax, float ymin, float ymax, File_Loc loc)
{
  std::cout << "CropWindow is unimplemented" << std::endl;
}

void Parser::Curves(const std::string &type,
                    std::vector<int> nvertices,
                    const std::string &wrap,
                    Parsed_Parameter_Vector params,
                    File_Loc loc)
{
  std::cout << "Curves is unimplemented" << std::endl;
}

void Parser::Cylinder(float radius,
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

void Parser::Declare(const std::string &name, const std::string &declaration, File_Loc loc)
{
  std::cout << "Declare is unimplemented" << std::endl;
}

void Parser::DepthOfField(float fstop, float focallength, float focaldistance, File_Loc loc)
{
  std::cout << "DepthOfField is unimplemented" << std::endl;
}

void Parser::Detail(float bound[6], File_Loc loc)
{
  std::cout << "Detail is unimplemented" << std::endl;
}

void Parser::DetailRange(float offlow, float onlow, float onhigh, float offhigh, File_Loc loc)
{
  std::cout << "DetailRange is unimplemented" << std::endl;
}

void Parser::Disk(
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

void Parser::Displacement(const std::string &displace,
                          const std::string &name,
                          Parsed_Parameter_Vector params,
                          File_Loc loc)
{
  std::cout << "Displacement is unimplemented" << std::endl;
}

void Parser::Display(const std::string &name,
                     const std::string &type,
                     const std::string &mode,
                     Parsed_Parameter_Vector params,
                     File_Loc loc)
{
  // If the first char of `name' is a '+', then it's an additional
  // channel to render. Ignore it for now.
  if (name[0] == '+') {
    fprintf(stdout, "Ignoring additional display channel, %s", name.c_str());
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

void Parser::DisplayChannel(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "DisplayChannel " << name << " is unimplemented" << std::endl;
}

void Parser::Else(File_Loc loc)
{
  std::cout << "Else is unimplemented" << std::endl;
}

void Parser::ElseIf(const std::string &condition, File_Loc loc)
{
  std::cout << "ElseIf is unimplemented" << std::endl;
}

void Parser::End(File_Loc loc)
{
  std::cout << "End is unimplemented" << std::endl;
}

void Parser::Exposure(float gain, float gamma, File_Loc loc)
{
  std::cout << "Exposure is unimplemented" << std::endl;
}

void Parser::Exterior(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Exterior is unimplemented" << std::endl;
}

void Parser::Format(int xresolution, int yresolution, float pixelaspectratio, File_Loc loc)
{
  std::cout << "Format is unimplemented" << std::endl;
}

void Parser::FrameAspectRatio(float frameratio, File_Loc loc)
{
  std::cout << "FrameAspectRatio is unimplemented" << std::endl;
}

void Parser::FrameBegin(int number, File_Loc loc)
{
  std::cout << "FrameBegin is unimplemented" << std::endl;
}

void Parser::FrameEnd(File_Loc loc)
{
  std::cout << "FrameEnd is unimplemented" << std::endl;
}

void Parser::GeneralPolygon(std::vector<int> nvertices,
                            Parsed_Parameter_Vector params,
                            File_Loc loc)
{
  std::cout << "GeneralPolygon is unimplemented" << std::endl;
}

void Parser::GeometricApproximation(const std::string &type, float value, File_Loc loc)
{
  std::cout << "GeometricApproximation is unimplemented" << std::endl;
}

void Parser::Geometry(const std::string &type, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Geometry is unimplemented" << std::endl;
}

void Parser::Hider(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Hider is unimplemented" << std::endl;
}

void Parser::Hyperboloid(
    Point3f point1, Point3f point2, float thetamax, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Hyperboloid is unimplemented" << std::endl;
}

void Parser::Identity(File_Loc loc)
{
  graphics_state.ctm = transform_identity();
}

void Parser::IfBegin(const std::string &condition, File_Loc loc)
{
  std::cout << "IfBegin is unimplemented" << std::endl;
}

void Parser::IfEnd(File_Loc loc)
{
  std::cout << "IfEnd is unimplemented" << std::endl;
}

void Parser::Illuminate(const std::string &light, bool onoff, File_Loc loc)
{
  std::cout << "Illuminate is unimplemented" << std::endl;
}

void Parser::Imager(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Imager is unimplemented" << std::endl;
}

void Parser::Integrator(const std::string &type,
                        const std::string &name,
                        Parsed_Parameter_Vector params,
                        File_Loc loc)
{
  std::cout << "Integrator is unimplemented" << std::endl;
}

void Parser::Interior(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Interior is unimplemented" << std::endl;
}

void Parser::Light(const std::string &name,
                   const std::string &handle,
                   Parsed_Parameter_Vector params,
                   File_Loc loc)
{
  VERIFY_WORLD("Light");
  Parsed_Parameter *exposure, *intensity, *light_color;
  Parsed_Parameter_Vector new_params, shape_params;

  for (auto it = params.begin(); it != params.end(); it++) {
    if ((*it)->name == "exposure")
      exposure = *it;
    else if ((*it)->name == "intensity")
      intensity = *it;
    else if ((*it)->name == "lightColor")
      light_color = *it;
    else
      new_params.push_back(*it);
  }

  Parsed_Parameter *param = new Parsed_Parameter(loc);
  float exposure_scale = 1.f;
  if (exposure)
    exposure_scale = pow(2.f, exposure->floats[0]);

  param->type = "rgb";
  param->name = "L";
  for (int i = 0; i < light_color->floats.size(); i++)
    param->add_float(exposure_scale * light_color->floats[i] * intensity->floats[0]);
  new_params.push_back(param);

  bool double_sided = false;
  for (auto it = graphics_state.shape_attributes.begin();
       it != graphics_state.shape_attributes.end();
       it++)
    if ((*it)->name == "sides") {
      double_sided = (*it)->floats[0] == 2;
      break;
    }

  // Lights are single-sided by default
  if (double_sided) {
    param = new Parsed_Parameter(loc);
    param->type = "bool";
    param->name = "twosided";
    param->may_be_unused = true;
    param->add_bool(true);
    new_params.push_back(param);
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
  graphics_state.area_light_params = Parameter_Dictionary(std::move(new_params),
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

void Parser::LightSource(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "LightSource is unimplemented." << std::endl;
}

void Parser::MakeBrickMap(std::vector<std::string> ptcnames,
                          const std::string &bkmname,
                          Parsed_Parameter_Vector params,
                          File_Loc loc)
{
  std::cout << "MakeBrickMap is unimplemented" << std::endl;
}

void Parser::MakeCubeFaceEnvironment(const std::string &px,
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

void Parser::MakeLatLongEnvironment(const std::string &picturename,
                                    const std::string &texturename,
                                    const std::string &filt,
                                    float swidth,
                                    float twidth,
                                    Parsed_Parameter_Vector params,
                                    File_Loc loc)
{
  std::cout << "MakeLatLongEnvironment is unimplemented" << std::endl;
}

void Parser::MakeShadow(const std::string &picturename,
                        const std::string &texturename,
                        Parsed_Parameter_Vector params,
                        File_Loc loc)
{
  std::cout << "MakeShadow is unimplemented" << std::endl;
}

void Parser::MakeTexture(const std::string &picturename,
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

void Parser::Matte(bool onoff, File_Loc loc)
{
  std::cout << "Matte is unimplemented" << std::endl;
}

void Parser::MotionBegin(std::vector<float> times, File_Loc loc)
{
  std::cout << "MotionBegin is unimplemented" << std::endl;
}

void Parser::MotionEnd(File_Loc loc)
{
  std::cout << "MotionEnd is unimplemented" << std::endl;
}

void Parser::NuPatch(int nu,
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

void Parser::ObjectBegin(const std::string &name, File_Loc loc)
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

void Parser::ObjectEnd(File_Loc loc)
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

void Parser::ObjectInstance(const std::string &name, File_Loc loc)
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

  Transform world_from_render = transform_inverse(render_from_world);

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

  Transform render_from_instance = render_from_world * graphics_state.ctm * world_from_render;
  instance_uses.push_back(Instance_Scene_Entity(name,
                                                loc,
                                                graphics_state.current_material_name,
                                                std::move(dict) /* RenderMan*/,
                                                render_from_instance));
}

void Parser::Option(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
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

void Parser::Opacity(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Opacity is unimplemented" << std::endl;
}

void Parser::Orientation(const std::string &orientation, File_Loc loc)
{
  std::cout << "Orientation is unimplemented" << std::endl;
}

void Parser::Paraboloid(float rmax,
                        float zmin,
                        float zmax,
                        float thetamax,
                        Parsed_Parameter_Vector params,
                        File_Loc loc)
{
  std::cout << "Paraboloid is unimplemented" << std::endl;
}

void Parser::Patch(const std::string &type, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Patch is unimplemented" << std::endl;
}

void Parser::PatchMesh(const std::string &type,
                       int nu,
                       const std::string &uwrap,
                       int nv,
                       const std::string &vwrap,
                       Parsed_Parameter_Vector params,
                       File_Loc loc)
{
  std::cout << "PatchMesh is unimplemented" << std::endl;
}

void Parser::Pattern(const std::string &name,
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

void Parser::Perspective(float fov, File_Loc loc)
{
  std::cout << "Perspective is unimplemented" << std::endl;
}

void Parser::PixelFilter(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "PixelFilter is unimplemented." << std::endl;
}

void Parser::PixelSampleImager(const std::string &name,
                               Parsed_Parameter_Vector params,
                               File_Loc loc)
{
  std::cout << "PixelSampleImager is unimplemented" << std::endl;
}

void Parser::PixelSamples(float xsamples, float ysamples, File_Loc loc)
{
  std::cout << "PixelSamples is unimplemented" << std::endl;
}

void Parser::PixelVariance(float variance, File_Loc loc)
{
  std::cout << "PixelVariance is unimplemented" << std::endl;
}

void Parser::Points(int npoints, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Points is unimplemented" << std::endl;
}

void Parser::PointsGeneralPolygons(std::vector<int> n_loops,
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

void Parser::PointsPolygons(std::vector<int> n_vertices,
                            std::vector<int> vertices,
                            Parsed_Parameter_Vector params,
                            File_Loc loc)
{
  bool needs_tessellation = false;
  bool mixed_polygons = false;
  int base_vert_count = n_vertices[0];
  for (auto &i : n_vertices)
    if (base_vert_count != i && i <= 4) {
      mixed_polygons = true;
    }
    else if (i > 4) {
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
    if (mixed_polygons) {
      fprintf(stdout,
              "Either the polygons have mixed number of vertices, or it has "
              "greater than 4 verts per poly");

      Parsed_Parameter_Vector quad_params;
      for (int i = 0; i < params.size(); ++i) {
        // Manually copy the parameters into the quad parameter vector as the
        // Inlined_Vector copy constructor does a shallow copy which causes a
        // double free later
        Parsed_Parameter *param = new Parsed_Parameter(*(params[i]));
        quad_params.push_back(param);
      }

      Parsed_Parameter *param3 = new Parsed_Parameter(loc);
      Parsed_Parameter *param4 = new Parsed_Parameter(loc);
      param3->type = "integer";
      param3->name = "indices";
      param4->type = "integer";
      param4->name = "indices";
      int index = 0;
      for (auto &i : n_vertices) {
        if (i == 3) {
          param3->add_int(vertices[index]);
          param3->add_int(vertices[index + 1]);
          param3->add_int(vertices[index + 2]);
        }
        else if (i == 4) {
          param4->add_int(vertices[index]);
          param4->add_int(vertices[index + 1]);
          param4->add_int(vertices[index + 3]);
          param4->add_int(vertices[index + 2]);
        }

        index += i;
      }

      if (param3->ints.size() > 0) {
        params.push_back(param3);
        Shape("trianglemesh", params, loc);
      }
      if (param4->ints.size() > 0) {
        quad_params.push_back(param4);
        Shape("bilinearmesh", quad_params, loc);
      }
    }
    else {
      Parsed_Parameter *param = new Parsed_Parameter(loc);
      param->type = "integer";
      param->name = "indices";
      if (base_vert_count == 3)
        for (int i = 0; i < vertices.size(); ++i)
          param->add_int(vertices[i]);
      else if (base_vert_count == 4)
        // Have to reorder last two vertices
        for (int i = 0; i < vertices.size(); i += 4) {
          param->add_int(vertices[i]);
          param->add_int(vertices[i + 1]);
          param->add_int(vertices[i + 3]);
          param->add_int(vertices[i + 2]);
        }
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

      if (base_vert_count == 3)
        Shape("trianglemesh", params, loc);
      else
        Shape("bilinearmesh", params, loc);
    }
  }
}

void Parser::Polygon(int nvertices, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Polygon is unimplemented" << std::endl;
}

void Parser::ProcDelayedReadArchive(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcDelayedReadArchive is unimplemented" << std::endl;
}

void Parser::ProcDynamicLoad(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcDynamicLoad is unimplemented" << std::endl;
}

void Parser::Procedural(const std::string &proc_name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Procedural is unimplemented" << std::endl;
}

void Parser::Procedural2(const std::string &proc_name,
                         Parsed_Parameter_Vector params,
                         File_Loc loc)
{
  std::cout << "Procedural2 is unimplemented" << std::endl;
}

void Parser::ProcFree(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcFree is unimplemented" << std::endl;
}

void Parser::ProcRunProgram(Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ProcRunProgram is unimplemented" << std::endl;
}

void Parser::Projection(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
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

void Parser::Quantize(
    const std::string &type, int one, int min, int max, float ditheramplitude, File_Loc loc)
{
  std::cout << "Quantize is unimplemented" << std::endl;
}

void Parser::ReadArchive(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "ReadArchive is unimplemented" << std::endl;
}

void Parser::RelativeDetail(float relativedetail, File_Loc loc)
{
  std::cout << "RelativeDetail is unimplemented" << std::endl;
}

void Parser::Resource(const std::string &handle,
                      const std::string &type,
                      Parsed_Parameter_Vector params,
                      File_Loc loc)
{
  std::cout << "Resource is unimplemented" << std::endl;
}

void Parser::ResourceBegin(File_Loc loc)
{
  std::cout << "ResourceBegin is unimplemented" << std::endl;
}

void Parser::ResourceEnd(File_Loc loc)
{
  std::cout << "ResourceEnd is unimplemented" << std::endl;
}

void Parser::ReverseOrientation(File_Loc loc)
{
  VERIFY_WORLD("ReverseOrientation");
  graphics_state.reverse_orientation = !graphics_state.reverse_orientation;
}

void Parser::Rotate(float angle, float ax, float ay, float az, File_Loc loc)
{
  graphics_state.ctm = graphics_state.ctm *
                       transform_rotate(DEG2RADF(angle), make_float3(ax, ay, az));
}

void Parser::Scale(float sx, float sy, float sz, File_Loc loc)
{
  graphics_state.ctm = graphics_state.ctm * transform_scale(make_float3(sx, sy, sz));
}

void Parser::ScopedCoordinateSystem(const std::string &, File_Loc loc)
{
  std::cout << "ScopedCoordinateSystem is unimplemented" << std::endl;
}

void Parser::ScreenWindow(float left, float right, float bottom, float top, File_Loc loc)
{
  std::cout << "ScreenWindow is unimplemented" << std::endl;
}

void Parser::ShadingInterpolation(const std::string &type, File_Loc loc)
{
  std::cout << "ShadingInterpolation is unimplemented" << std::endl;
}

void Parser::ShadingRate(float size, File_Loc loc)
{
  std::cout << "ShadingRate is unimplemented" << std::endl;
}

void Parser::Shutter(float opentime, float closetime, File_Loc loc)
{
  std::cout << "Shutter is unimplemented" << std::endl;
}

void Parser::Sides(int nsides, File_Loc loc)
{
  Parsed_Parameter *param = new Parsed_Parameter(loc);
  param->type = "integer";
  param->name = "sides";
  param->add_float(nsides);
  graphics_state.shape_attributes.push_back(param);
}

void Parser::Skew(
    float angle, float dx1, float dy1, float dz1, float dx2, float dy2, float dz2, File_Loc loc)
{
  std::cout << "Skew is unimplemented" << std::endl;
}

void Parser::SolidBegin(const std::string &type, File_Loc loc)
{
  std::cout << "SolidBegin is unimplemented" << std::endl;
}

void Parser::SolidEnd(File_Loc loc)
{
  std::cout << "SolidEnd is unimplemented" << std::endl;
}

void Parser::Sphere(float radius,
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

void Parser::SubdivisionMesh(const std::string &scheme,
                             int nfaces,
                             Parsed_Parameter_Vector params,
                             File_Loc loc)
{
  std::cout << "SubdivisionMesh is unimplemented" << std::endl;
}

void Parser::Surface(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  std::cout << "Surface is unimplemented" << std::endl;
}

void Parser::System(const std::string &cmd, File_Loc loc)
{
  std::cout << "System is unimplemented" << std::endl;
}

void Parser::Texture(const std::string &name,
                     const std::string &type,
                     const std::string &texname,
                     Parsed_Parameter_Vector params,
                     File_Loc loc)
{
  std::cout << "Texture is unimplemented." << std::endl;
}

void Parser::TextureCoordinates(
    float s1, float t1, float s2, float t2, float s3, float t3, float s4, float t4, File_Loc loc)
{
  std::cout << "TextureCoordinates is unimplemented" << std::endl;
}

void Parser::Torus(float majorrad,
                   float minorrad,
                   float phimin,
                   float phimax,
                   float thetamax,
                   Parsed_Parameter_Vector params,
                   File_Loc loc)
{
  std::cout << "Torus is unimplemented" << std::endl;
}

void Parser::transform(float transform[16], File_Loc loc)
{
  // Stomp the current transform
  ProjectionTransform projection = *(ProjectionTransform *)&transform[0];
  graphics_state.ctm = projection_to_transform(projection_transpose(projection));
}

void Parser::TransformBegin(File_Loc loc)
{
  std::cout << "TransformBegin is unimplemented" << std::endl;
}

void Parser::TransformEnd(File_Loc loc)
{
  std::cout << "TransformEnd is unimplemented" << std::endl;
}

void Parser::Translate(float dx, float dy, float dz, File_Loc loc)
{
  graphics_state.ctm = graphics_state.ctm * transform_translate(make_float3(dx, dy, dz));
}

void Parser::TrimCurve(int nloops,
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

void Parser::WorldBegin(File_Loc loc)
{
  VERIFY_OPTIONS("WorldBegin");
  // Reset graphics state for _WorldBegin_
  current_block = Block_State::World_Block;
  graphics_state.ctm = transform_identity();
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
  opt_param = options["shader_default"];
  paths += opt_param->strings[0];
  // g_options->shader_search_paths = paths;

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

void Parser::WorldEnd(File_Loc loc)
{
  std::cout << "WorldEnd is unimplemented" << std::endl;
}

void Parser::end_of_files()
{
  // Do end of parsing operations
}

void Parser::Shape(const std::string &name, Parsed_Parameter_Vector params, File_Loc loc)
{
  VERIFY_WORLD("Shape");

  Parameter_Dictionary dict(std::move(params), graphics_state.shape_attributes);

  int area_light_index = -1;
  if (!graphics_state.area_light_name.empty()) {
    area_light_index = add_area_light(Scene_Entity(graphics_state.area_light_name,
                                                   graphics_state.area_light_params,
                                                   graphics_state.area_light_loc));

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
    Transform render_from_object = graphics_state.ctm;
    Transform object_from_render = transform_inverse(graphics_state.ctm);

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
}

Parser::OSL_Shader Parser::bxdf_to_osl(std::string bxdf,
                                       std::string name,
                                       Parsed_Parameter_Vector &params)
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

void Parser::add_light(Light_Scene_Entity light)
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

int Parser::add_area_light(Scene_Entity light)
{
  std::lock_guard<std::mutex> lock(area_light_mutex);
  area_lights.push_back(std::move(light));
  return area_lights.size() - 1;
}

void Parser::add_shapes(p_std::span<Shape_Scene_Entity> s)
{
  std::lock_guard<std::mutex> lock(shape_mutex);
  std::move(std::begin(s), std::end(s), std::back_inserter(shapes));
}

void Parser::add_animated_shape(Animated_Shape_Scene_Entity shape)
{
  std::lock_guard<std::mutex> lock(animated_shape_mutex);
  animated_shapes.push_back(std::move(shape));
}

void Parser::add_instance_definition(Instance_Definition_Scene_Entity instance)
{
  Instance_Definition_Scene_Entity *def = new Instance_Definition_Scene_Entity(
      std::move(instance));

  std::lock_guard<std::mutex> lock(instance_definition_mutex);
  instance_definitions[def->name] = def;
}

void Parser::add_instance_uses(p_std::span<Instance_Scene_Entity> in)
{
  std::lock_guard<std::mutex> lock(instance_use_mutex);
  std::move(std::begin(in), std::end(in), std::back_inserter(instances));
}

static std::string to_string_from_view(std::string_view s)
{
  return std::string(s.data(), s.size());
}

std::string Token::to_string() const
{
  return to_string_from_view(token);
}

// Tokenizer Implementation
static char decode_escaped(int ch, const File_Loc &loc)
{
  switch (ch) {
    case EOF:
      exit(-1);
    case 'b':
      return '\b';
    case 'f':
      return '\f';
    case 'n':
      return '\n';
    case 'r':
      return '\r';
    case 't':
      return '\t';
    case '\\':
      return '\\';
    case '\'':
      return '\'';
    case '\"':
      return '\"';
    default:
      exit(-1);
  }
  return 0;  // NOTREACHED
}

std::string read_file_contents(std::string filename)
{
#ifdef IS_WINDOWS
  std::ifstream ifs(WStringFromUTF8(filename).c_str(), std::ios::binary);
  if (!ifs)
    error_exit("%s: %s", filename, error_string());
  return std::string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
#else
  int fd = open(filename.c_str(), O_RDONLY);
  if (fd == -1)
    exit(-1);

  struct stat stat;
  if (fstat(fd, &stat) != 0)
    exit(-1);

  std::string contents(stat.st_size, '\0');
  if (read(fd, contents.data(), stat.st_size) == -1)
    exit(-1);

  close(fd);
  return contents;
#endif
}

static double_conversion::StringToDoubleConverter floatParser(
    double_conversion::StringToDoubleConverter::ALLOW_HEX,
    0. /* empty string value */,
    0. /* junk string value */,
    nullptr /* infinity symbol */,
    nullptr /* NaN symbol */);

std::unique_ptr<Tokenizer> Tokenizer::create_from_file(
    const std::string &filename,
    std::function<void(const char *, const File_Loc *)> error_callback)
{
  if (filename == "-") {
    // Handle stdin by slurping everything into a string.
    std::string str;
    int ch;
    while ((ch = getchar()) != EOF)
      str.push_back((char)ch);
    return std::make_unique<Tokenizer>(std::move(str), "<stdin>", std::move(error_callback));
  }

  // if ( has_extension( filename, ".gz" ) )
  //{
  //   std::string str = read_decompressed_file_contents( filename );
  //   return std::make_unique< Tokenizer >( std::move( str ), filename,
  //                                         std::move( error_callback ) );
  //}

#ifdef HAVE_MMAP
  int fd = open(filename.c_str(), O_RDONLY);
  if (fd == -1) {
    return nullptr;
  }

  struct stat stat;
  if (fstat(fd, &stat) != 0) {
    return nullptr;
  }

  size_t len = stat.st_size;
  if (len < 16 * 1024 * 1024) {
    close(fd);

    std::string str = read_file_contents(filename);
    return std::make_unique<Tokenizer>(std::move(str), filename, std::move(error_callback));
  }

  void *ptr = mmap(nullptr, len, PROT_READ, MAP_PRIVATE | MAP_NORESERVE, fd, 0);
  if (ptr == MAP_FAILED)

    if (close(fd) != 0) {
      return nullptr;
    }

  return std::make_unique<Tokenizer>(ptr, len, filename, std::move(error_callback));
#elif defined(IS_WINDOWS)
  auto errorReportLambda = [&error_callback, &filename]() -> std::unique_ptr<Tokenizer> {
    LPSTR messageBuffer = nullptr;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                       FORMAT_MESSAGE_IGNORE_INSERTS,
                   NULL,
                   ::GetLastError(),
                   MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                   (LPSTR)&messageBuffer,
                   0,
                   NULL);

    LocalFree(messageBuffer);
    return nullptr;
  };

  HANDLE fileHandle = CreateFileW(wstring_from_utf8(filename).c_str(),
                                  GENERIC_READ,
                                  FILE_SHARE_READ,
                                  0,
                                  OPEN_EXISTING,
                                  FILE_ATTRIBUTE_NORMAL,
                                  0);
  if (!fileHandle)
    return errorReportLambda();

  size_t len = GetFileSize(fileHandle, 0);

  HANDLE mapping = CreateFileMapping(fileHandle, 0, PAGE_READONLY, 0, 0, 0);
  CloseHandle(fileHandle);
  if (mapping == 0)
    return errorReportLambda();

  LPVOID ptr = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0);
  CloseHandle(mapping);
  if (!ptr) {
    return errorReportLambda();
  }

  return std::make_unique<Tokenizer>(ptr, len, filename, std::move(error_callback));
#else
  std::string str = read_file_contents(filename);
  return std::make_unique<Tokenizer>(std::move(str), filename, std::move(error_callback));
#endif
}

std::unique_ptr<Tokenizer> Tokenizer::create_from_string(
    std::string str, std::function<void(const char *, const File_Loc *)> error_callback)
{
  return std::make_unique<Tokenizer>(std::move(str), "<stdin>", std::move(error_callback));
}

Tokenizer::Tokenizer(std::string str,
                     std::string filename,
                     std::function<void(const char *, const File_Loc *)> error_callback)
    : error_callback(std::move(error_callback)), contents(std::move(str))
{
  loc = File_Loc(*new std::string(filename));
  pos = contents.data();
  end = pos + contents.size();
  check_utf(contents.data(), contents.size());
}

#if defined(HAVE_MMAP) || defined(IS_WINDOWS)
Tokenizer::Tokenizer(void *ptr,
                     size_t len,
                     std::string filename,
                     std::function<void(const char *, const File_Loc *)> error_callback)
    : error_callback(std::move(error_callback)), unmapPtr(ptr), unmapLength(len)
{
  // This is disgusting and leaks memory, but it ensures that the
  // filename in File_Locs returned by the Tokenizer remain valid even
  // after it has been destroyed.
  loc = File_Loc(*new std::string(filename));
  pos = (const char *)ptr;
  end = pos + len;
  check_utf(ptr, len);
}
#endif

Tokenizer::~Tokenizer()
{
#ifdef HAVE_MMAP
  if (unmapPtr && unmapLength > 0)
    if (munmap(unmapPtr, unmapLength) != 0)
      error_callback(string_printf("munmap: %s", error_string()).c_str(), nullptr);
#elif defined(IS_WINDOWS)
  if (unmapPtr && UnmapViewOfFile(unmapPtr) == 0)
    error_callback(string_printf("UnmapViewOfFile: %s", error_string()).c_str(), nullptr);
#endif
}

void Tokenizer::check_utf(const void *ptr, int len) const
{
  const unsigned char *c = (const unsigned char *)ptr;
  // https://en.wikipedia.org/wiki/Byte_order_mark
  if (len >= 2 && ((c[0] == 0xfe && c[1] == 0xff) || (c[0] == 0xff && c[1] == 0xfe)))
    error_callback(
        "File is encoded with UTF-16, which is not currently "
        "supported by pbrt (https://github.com/mmp/pbrt-v4/issues/136).",
        &loc);
}

std::optional<Token> Tokenizer::Next()
{
  while (true) {
    const char *tokenStart = pos;
    File_Loc startLoc = loc;

    int ch = get_char();
    if (ch == EOF)
      return {};
    else if (ch == ' ' || ch == '\n' || ch == '\t' || ch == '\r') {
      // nothing
    }
    else if (ch == '"') {
      // scan to closing quote
      bool haveEscaped = false;
      while ((ch = get_char()) != '"') {
        if (ch == EOF) {
          error_callback("premature EOF", &startLoc);
          return {};
        }
        else if (ch == '\n') {
          error_callback("unterminated string", &startLoc);
          return {};
        }
        else if (ch == '\\') {
          haveEscaped = true;
          // Grab the next character
          if ((ch = get_char()) == EOF) {
            error_callback("premature EOF", &startLoc);
            return {};
          }
        }
      }

      if (!haveEscaped)
        return Token({tokenStart, size_t(pos - tokenStart)}, startLoc);
      else {
        sEscaped.clear();
        for (const char *p = tokenStart; p < pos; ++p) {
          if (*p != '\\')
            sEscaped.push_back(*p);
          else {
            ++p;
            // CHECK_LT( p, pos );
            sEscaped.push_back(decode_escaped(*p, startLoc));
          }
        }
        return Token({sEscaped.data(), sEscaped.size()}, startLoc);
      }
    }
    else if (ch == '[' || ch == ']') {
      return Token({tokenStart, size_t(1)}, startLoc);
    }
    else if (ch == '#') {
      // comment: scan to EOL (or EOF)
      while ((ch = get_char()) != EOF) {
        if (ch == '\n' || ch == '\r') {
          unget_char();
          break;
        }
      }

      return Token({tokenStart, size_t(pos - tokenStart)}, startLoc);
    }
    else {
      // Regular statement or numeric token; scan until we hit a
      // space, opening quote, or bracket.
      while ((ch = get_char()) != EOF) {
        if (ch == ' ' || ch == '\n' || ch == '\t' || ch == '\r' || ch == '"' || ch == '[' ||
            ch == ']') {
          unget_char();
          break;
        }
      }
      return Token({tokenStart, size_t(pos - tokenStart)}, startLoc);
    }
  }
}

static int parse_int(const Token &t)
{
  bool negate = t.token[0] == '-';

  int index = 0;
  if (t.token[0] == '+' || t.token[0] == '-')
    ++index;

  int64_t value = 0;
  while (index < t.token.size()) {
    if (!(t.token[index] >= '0' && t.token[index] <= '9'))
      exit(-1);
    // error_exit( &t.loc, "\"%c\": expected a number", t.token[ index ] );
    value = 10 * value + (t.token[index] - '0');
    ++index;

    if (value > std::numeric_limits<int>::max())
      exit(-1);
    // error_exit(
    //    &t.loc,
    //    "Numeric value too large to represent as a 32-bit integer." );
    else if (value < std::numeric_limits<int>::lowest())
      std::cout << "Numeric value %d too low to represent as a 32-bit integer." << std::endl;
  }

  return negate ? -value : value;
}

static double parse_float(const Token &t)
{
  // Fast path for a single digit
  if (t.token.size() == 1) {
    if (!(t.token[0] >= '0' && t.token[0] <= '9'))
      exit(-1);
    // error_exit( &t.loc, "\"%c\": expected a number", t.token[ 0 ] );
    return t.token[0] - '0';
  }

  // Copy to a buffer so we can NUL-terminate it, as strto[idf]() expect.
  char buf[64];
  char *bufp = buf;
  std::unique_ptr<char[]> allocBuf;
  // CHECK_RARE( 1e-5, t.token.size() + 1 >= sizeof( buf ) );
  if (t.token.size() + 1 >= sizeof(buf)) {
    // This should be very unusual, but is necessary in case we get a
    // goofball number with lots of leading zeros, for example.
    allocBuf = std::make_unique<char[]>(t.token.size() + 1);
    bufp = allocBuf.get();
  }

  std::copy(t.token.begin(), t.token.end(), bufp);
  bufp[t.token.size()] = '\0';

  // Can we just use strtol?
  auto isInteger = [](std::string_view str) {
    for (char ch : str)
      if (!(ch >= '0' && ch <= '9'))
        return false;
    return true;
  };

  int length = 0;
  double val;
  if (isInteger(t.token)) {
    char *endptr;
    val = double(strtol(bufp, &endptr, 10));
    length = endptr - bufp;
  }
  else if (sizeof(float) == sizeof(float))
    val = (double)floatParser.StringToFloat(bufp, t.token.size(), &length);
  else
    val = floatParser.StringToDouble(bufp, t.token.size(), &length);

  if (length == 0)
    exit(-1);
  // error_exit( &t.loc, "%s: expected a number",
  //            to_string_from_view( t.token ) );

  return val;
}

inline bool is_quoted_string(std::string_view str)
{
  return str.size() >= 2 && str[0] == '"' && str.back() == '"';
}

static std::string_view dequote_string(const Token &t)
{
  if (!is_quoted_string(t.token))
    exit(-1);
  // error_exit( &t.loc, "\"%s\": expected quoted string",
  //            to_string_from_view( t.token ) );

  std::string_view str = t.token;
  str.remove_prefix(1);
  str.remove_suffix(1);
  return str;
}

constexpr int TokenOptional = 0;
constexpr int TokenRequired = 1;

template<typename Next, typename Unget>
static Parsed_Parameter_Vector parse_parameters(
    Next nextToken,
    Unget ungetToken,
    const std::function<void(const Token &token, const char *)> &error_callback)
{
  Parsed_Parameter_Vector parameterVector;

  while (true) {
    std::optional<Token> t = nextToken(TokenOptional);
    if (!t.has_value())
      return parameterVector;

    if (!is_quoted_string(t->token)) {
      ungetToken(*t);
      return parameterVector;
    }

    Parsed_Parameter *param = new Parsed_Parameter(t->loc);

    std::string_view decl = dequote_string(*t);

    auto skipSpace = [&decl](std::string_view::const_iterator iter) {
      while (iter != decl.end() && (*iter == ' ' || *iter == '\t'))
        ++iter;
      return iter;
    };
    // Skip to the next whitespace character (or the end of the string).
    auto skipToSpace = [&decl](std::string_view::const_iterator iter) {
      while (iter != decl.end() && *iter != ' ' && *iter != '\t')
        ++iter;
      return iter;
    };

    auto typeBegin = skipSpace(decl.begin());
    if (typeBegin == decl.end()) {
      std::cerr << &t->loc << "Parameter \"" << std::string(decl.begin(), decl.end())
                << "\" doesn't have a type declaration?!" << std::endl;
      exit(-1);
    }

    // Find end of type declaration
    auto typeEnd = skipToSpace(typeBegin);
    param->type.assign(typeBegin, typeEnd);

    // RenderMan
    bool was_storage = false;
    // Check to see if it's a container type
    if (param->type == "constant") {
      param->storage = Container_Type::Constant;
      was_storage = true;
    }
    else if (param->type == "facevarying") {
      param->storage = Container_Type::FaceVarying;
      was_storage = true;
    }
    else if (param->type == "reference") {
      param->storage = Container_Type::Reference;
      was_storage = true;
    }
    else if (param->type == "uniform") {
      param->storage = Container_Type::Uniform;
      was_storage = true;
    }
    else if (param->type == "varying") {
      param->storage = Container_Type::Varying;
      was_storage = true;
    }
    else if (param->type == "vertex") {
      param->storage = Container_Type::Vertex;
      was_storage = true;
    }

    if (was_storage) {
      typeBegin = skipSpace(typeEnd);
      if (typeBegin == decl.end()) {
        std::cerr << &t->loc << "Parameter \"" << std::string(decl.begin(), decl.end())
                  << "\" doesn't have a type declaration?!" << std::endl;
        exit(-1);
      }

      // Find end of type declaration
      typeEnd = skipToSpace(typeBegin);
      param->type.assign(typeBegin, typeEnd);
    }
    // See if it's an array
    std::vector<std::string> strings = split_string(param->type, '[');
    if (strings.size() > 1)
      // Could extract the size and check we have that many values below
      param->type = strings[0];

    // Convert to PBRT type
    if (param->type == "int")
      param->type = "integer";
    else if (param->type == "color")
      param->type = "rgb";
    else if (param->type == "point")
      param->type = "point3";
    else if (param->type == "vector")
      param->type = "vector3";

    auto nameBegin = skipSpace(typeEnd);
    if (nameBegin == decl.end()) {
      std::cerr << &t->loc << " Unable to find parameter name from \""
                << std::string(decl.begin(), decl.end()) << "\"" << std::endl;
      exit(-1);
    }

    auto nameEnd = skipToSpace(nameBegin);
    param->name.assign(nameBegin, nameEnd);

    enum ValType { Unknown, String, Bool, Float, Int } valType = Unknown;

    if (param->type == "integer")
      valType = Int;

    auto addVal = [&](const Token &t) {
      if (is_quoted_string(t.token)) {
        switch (valType) {
          case Unknown:
            valType = String;
            break;
          case String:
            break;
          case Float:
            error_callback(t, "expected floating-point value");
          case Int:
            error_callback(t, "expected integer value");
          case Bool:
            error_callback(t, "expected Boolean value");
        }

        param->add_string(dequote_string(t));
      }
      else if (t.token[0] == 't' && t.token == "true") {
        switch (valType) {
          case Unknown:
            valType = Bool;
            break;
          case String:
            error_callback(t, "expected string value");
          case Float:
            error_callback(t, "expected floating-point value");
          case Int:
            error_callback(t, "expected integer value");
          case Bool:
            break;
        }

        param->add_bool(true);
      }
      else if (t.token[0] == 'f' && t.token == "false") {
        switch (valType) {
          case Unknown:
            valType = Bool;
            break;
          case String:
            error_callback(t, "expected string value");
          case Float:
            error_callback(t, "expected floating-point value");
          case Int:
            error_callback(t, "expected integer value");
          case Bool:
            break;
        }

        param->add_bool(false);
      }
      else {
        switch (valType) {
          case Unknown:
            valType = Float;
            break;
          case String:
            error_callback(t, "expected string value");
          case Float:
            break;
          case Int:
            break;
          case Bool:
            error_callback(t, "expected Boolean value");
        }

        if (valType == Int)
          param->add_int(parse_int(t));
        else
          param->add_float(parse_float(t));
      }
    };

    Token val = *nextToken(TokenRequired);

    if (val.token == "[") {
      while (true) {
        val = *nextToken(TokenRequired);
        if (val.token == "]")
          break;
        addVal(val);
      }
    }
    else {
      addVal(val);
    }

    // RenderMan
    param->may_be_unused = true;

    parameterVector.push_back(param);
  }

  return parameterVector;
}

void parse(Parser *target, std::unique_ptr<Tokenizer> t)
{
  static std::atomic<bool> warned_transform_begin_end_deprecated = false;

  // LOG_VERBOSE( "Started parsing %s",
  //             std::string( t->loc.filename.begin(), t->loc.filename.end() ) );

  std::vector<std::unique_ptr<Tokenizer>> fileStack;
  fileStack.push_back(std::move(t));

  std::optional<Token> ungetToken;

  auto parse_error = [&](const char *msg, const File_Loc *loc) {
    std::cerr << loc << " " << msg << std::endl;
    exit(-2);
  };

  // nextToken is a little helper function that handles the file stack,
  // returning the next token from the current file until reaching EOF,
  // at which point it switches to the next file (if any).
  std::function<std::optional<Token>(int)> nextToken;
  nextToken = [&](int flags) -> std::optional<Token> {
    if (ungetToken.has_value())
      return std::exchange(ungetToken, {});

    if (fileStack.empty()) {
      if ((flags & TokenRequired) != 0) {
        std::cerr << "premature end of file" << std::endl;
        exit(-12);
      }
      return {};
    }

    std::optional<Token> tok = fileStack.back()->Next();

    if (!tok) {
      // We've reached EOF in the current file. Anything more to parse?
      // LOG_VERBOSE( "Finished parsing %s",
      //              std::string( fileStack.back()->loc.filename.begin(),
      //                           fileStack.back()->loc.filename.end() ) );
      fileStack.pop_back();
      return nextToken(flags);
    }
    else if (tok->token[0] == '#') {
      // Swallow comments, unless --format or --toply was given, in
      // which case they're printed to stdout.
      return nextToken(flags);
    }
    else
      // Regular token; success.
      return tok;
  };

  auto unget = [&](Token t) {
    // CHECK( !ungetToken.has_value() );
    ungetToken = t;
  };

  // Helper function for pbrt API entrypoints that take a single string
  // parameter and a ParameterVector (e.g. pbrtShape()).
  auto basic_param_list_entrypoint =
      [&](void (Parser::*apiFunc)(const std::string &, Parsed_Parameter_Vector, File_Loc),
          File_Loc loc) {
        Token t = *nextToken(TokenRequired);
        std::string_view dequoted = dequote_string(t);
        std::string n = to_string_from_view(dequoted);
        Parsed_Parameter_Vector parameterVector = parse_parameters(
            nextToken, unget, [&](const Token &t, const char *msg) {
              std::string token = to_string_from_view(t.token);
              std::string str = msg;
              parse_error(str.c_str(), &t.loc);
            });
        (target->*apiFunc)(n, std::move(parameterVector), loc);
      };

  // Helper function for RenderMan API entrypoints that take two string
  // parameters and a ParameterVector (e.g. Bxdf()).
  // using two_string_param_list_entrypoint = void (Basic_Scene_Builder::*)(const
  // std::string &, const std::string &, Parsed_Parameter_Vector, File_Loc);
  auto two_string_param_list_entrypoint =
      [&](void (Parser::*apiFunc)(
              const std::string &, const std::string &, Parsed_Parameter_Vector, File_Loc),
          File_Loc loc) {
        Token t = *nextToken(TokenRequired);
        std::string_view dequoted = dequote_string(t);
        std::string n = to_string_from_view(dequoted);
        t = *nextToken(TokenRequired);
        dequoted = dequote_string(t);
        std::string p = to_string_from_view(dequoted);
        Parsed_Parameter_Vector parameterVector = parse_parameters(
            nextToken, unget, [&](const Token &t, const char *msg) {
              std::string token = to_string_from_view(t.token);
              std::string str = msg;
              parse_error(str.c_str(), &t.loc);
            });
        (target->*apiFunc)(n, p, std::move(parameterVector), loc);
      };

  auto syntax_error = [&](const Token &t) {
    std::cerr << &t.loc << " Unknown directive: " << to_string_from_view(t.token) << std::endl;
    exit(-3);
    ;
  };

  auto parse_array_of_real = [&](std::vector<float> vals) {
    Token val = *nextToken(TokenRequired);

    if (val.token == "[") {
      while (true) {
        val = *nextToken(TokenRequired);
        if (val.token == "]")
          break;
        vals.push_back(parse_float(val));
      }
    }
    else
      syntax_error(val);
  };

  auto parse_array_of_int = [&](std::vector<int> &vals) {
    Token val = *nextToken(TokenRequired);

    if (val.token == "[") {
      while (true) {
        val = *nextToken(TokenRequired);
        if (val.token == "]")
          break;
        vals.push_back((int)(parse_int(val)));
      }
    }
    else
      syntax_error(val);
  };

  std::optional<Token> tok;

  while (true) {
    tok = nextToken(TokenOptional);
    if (!tok.has_value())
      break;

    switch (tok->token[0]) {
      case 'A':
        if (tok->token == "AttributeBegin")
          target->AttributeBegin(tok->loc);
        else if (tok->token == "AttributeEnd")
          target->AttributeEnd(tok->loc);
        else if (tok->token == "Attribute")
          basic_param_list_entrypoint(&Parser::Attribute, tok->loc);
        else if (tok->token == "AreaLightSource")
          basic_param_list_entrypoint(&Parser::AreaLightSource, tok->loc);
        // Ri API
        else if (tok->token == "Atmosphere")
          basic_param_list_entrypoint(&Parser::Atmosphere, tok->loc);
        else
          syntax_error(*tok);
        break;

      // Ri API
      case 'B':
        if (tok->token == "Basis") {
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Basis(std::move(params), tok->loc);
        }
        else if (tok->token == "Begin") {
          std::string_view n = dequote_string(*nextToken(TokenRequired));
          target->Begin(to_string_from_view(n), tok->loc);
        }
        else if (tok->token == "Bound") {
          bool bracketed = false;
          std::optional<Token> second = nextToken(TokenOptional);
          if (second.has_value()) {
            if (second->token == "[")
              bracketed = true;
            else
              syntax_error(*tok);
          }

          float m[6];
          for (int i = 0; i < 6; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          if (bracketed)
            if (nextToken(TokenRequired)->token != "]")
              syntax_error(*tok);
          target->Bound(m, tok->loc);
        }
        else if (tok->token == "Bxdf")
          two_string_param_list_entrypoint(&Parser::Bxdf, tok->loc);
        else
          syntax_error(*tok);
        break;

      case 'C':
        if (tok->token == "ConcatTransform") {
          if (nextToken(TokenRequired)->token != "[")
            syntax_error(*tok);
          float m[16];
          for (int i = 0; i < 16; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          if (nextToken(TokenRequired)->token != "]")
            syntax_error(*tok);
          target->ConcatTransform(m, tok->loc);
        }
        else if (tok->token == "CoordinateSystem") {
          std::string_view n = dequote_string(*nextToken(TokenRequired));
          target->CoordinateSystem(to_string_from_view(n), tok->loc);
        }
        else if (tok->token == "CoordSysTransform") {
          std::string_view n = dequote_string(*nextToken(TokenRequired));
          target->CoordSysTransform(to_string_from_view(n), tok->loc);
        }
        else if (tok->token == "Camera")
          basic_param_list_entrypoint(&Parser::camera, tok->loc);
        // Ri API
        else if (tok->token == "Clipping") {
          float m[2];
          for (int i = 0; i < 2; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          target->Clipping(m[0], m[1], tok->loc);
        }
        else if (tok->token == "ClippingPlane") {
          float m[6];
          for (int i = 0; i < 6; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          target->ClippingPlane(m[0], m[1], m[2], m[3], m[4], m[5], tok->loc);
        }
        else if (tok->token == "Color") {
          float m[3];
          for (int i = 0; i < 3; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          target->Color(m[0], m[1], m[2], tok->loc);
        }
        else if (tok->token == "Cone") {
          bool bracketed = false;
          std::optional<Token> second = nextToken(TokenOptional);
          if (second.has_value()) {
            if (second->token == "[")
              bracketed = true;
            else
              syntax_error(*tok);
          }

          float m[3];
          for (int i = 0; i < 3; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));

          if (bracketed)
            if (nextToken(TokenRequired)->token != "]")
              syntax_error(*tok);
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Cone(m[0], m[1], m[2], std::move(params), tok->loc);
        }
        else if (tok->token == "CropWindow") {
          float m[4];
          for (int i = 0; i < 4; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          target->CropWindow(m[0], m[1], m[2], m[3], tok->loc);
        }
        else if (tok->token == "Curves") {
          std::string_view type = dequote_string(*nextToken(TokenRequired));

          std::vector<int> n_vertices;
          parse_array_of_int(n_vertices);

          std::string_view wrap = dequote_string(*nextToken(TokenRequired));
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Curves(to_string_from_view(type),
                         n_vertices,
                         to_string_from_view(wrap),
                         std::move(params),
                         tok->loc);
        }
        else if (tok->token == "Cylinder") {
          bool bracketed = false;
          std::optional<Token> second = nextToken(TokenOptional);

          if (second.has_value()) {
            if (second->token == "[")
              bracketed = true;
            else
              unget(*second);
          }

          float m[4];
          for (int i = 0; i < 4; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));

          if (bracketed)
            if (nextToken(TokenRequired)->token != "]")
              syntax_error(*tok);
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Cylinder(m[0], m[1], m[2], m[3], std::move(params), tok->loc);
        }
        else
          syntax_error(*tok);
        break;

      // RI API
      case 'D':
        if (tok->token == "Declare") {
          std::string name = to_string_from_view(dequote_string(*nextToken(TokenRequired)));
          std::string decl = to_string_from_view(dequote_string(*nextToken(TokenRequired)));
          target->Declare(name, decl, tok->loc);
        }
        else if (tok->token == "DepthOfField") {
          float m[3];
          for (int i = 0; i < 3; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          target->DepthOfField(m[0], m[1], m[2], tok->loc);
        }
        else if (tok->token == "Detail") {
          float m[6];
          for (int i = 0; i < 6; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          target->Detail(m, tok->loc);
        }
        else if (tok->token == "DetailRange") {
          bool bracketed = false;
          std::optional<Token> second = nextToken(TokenOptional);
          if (second.has_value()) {
            if (second->token == "[")
              bracketed = true;
            else
              syntax_error(*tok);
          }

          float m[4];
          for (int i = 0; i < 4; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));

          if (bracketed)
            if (nextToken(TokenRequired)->token != "]")
              syntax_error(*tok);
          target->DetailRange(m[0], m[1], m[2], m[3], tok->loc);
        }
        else if (tok->token == "Disk") {
          bool bracketed = false;
          std::optional<Token> second = nextToken(TokenOptional);
          if (second.has_value()) {
            if (second->token == "[")
              bracketed = true;
            else
              syntax_error(*tok);
          }

          float m[3];
          for (int i = 0; i < 3; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));

          if (bracketed)
            if (nextToken(TokenRequired)->token != "]")
              syntax_error(*tok);
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Disk(m[0], m[1], m[2], std::move(params), tok->loc);
        }
        else if (tok->token == "Displace")
          two_string_param_list_entrypoint(&Parser::Displacement, tok->loc);
        else if (tok->token == "Display") {
          std::string_view name = dequote_string(*nextToken(TokenRequired));
          std::string_view type = dequote_string(*nextToken(TokenRequired));
          std::string_view mode = dequote_string(*nextToken(TokenRequired));
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Display(to_string_from_view(name),
                          to_string_from_view(type),
                          to_string_from_view(mode),
                          std::move(params),
                          tok->loc);
        }
        else if (tok->token == "DisplayChannel") {
          std::string_view channel = dequote_string(*nextToken(TokenRequired));
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->DisplayChannel(to_string_from_view(channel), std::move(params), tok->loc);
        }
        else
          syntax_error(*tok);
        break;

      // RI API
      case 'E':
        if (tok->token == "Else")
          target->Else(tok->loc);
        else if (tok->token == "ElseIf") {
          std::string_view expression = dequote_string(*nextToken(TokenRequired));
          target->ElseIf(to_string_from_view(expression), tok->loc);
        }
        else if (tok->token == "End")
          target->End(tok->loc);
        else if (tok->token == "Exposure") {
          float m[2];
          for (int i = 0; i < 2; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));

          target->Exposure(m[0], m[1], tok->loc);
        }
        else if (tok->token == "Exterior") {
          basic_param_list_entrypoint(&Parser::Atmosphere, tok->loc);
        }
        else
          syntax_error(*tok);
        break;

      case 'F':
        if (tok->token == "Format") {
          int m[3];
          for (int i = 0; i < 3; ++i)
            m[i] = (int)parse_float(*nextToken(TokenRequired));

          target->Format(m[0], m[1], m[2], tok->loc);
        }
        else if (tok->token == "FrameAspectRatio")
          target->FrameAspectRatio(parse_float(*nextToken(TokenRequired)), tok->loc);
        else if (tok->token == "FrameBegin")
          target->FrameBegin((int)(parse_float(*nextToken(TokenRequired))), tok->loc);
        else if (tok->token == "FrameEnd")
          target->FrameEnd(tok->loc);
        else
          syntax_error(*tok);
        break;

      // RI API
      case 'G':
        if (tok->token == "GeneralPolygon") {
          std::vector<int> n_vertices;
          parse_array_of_int(n_vertices);

          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->GeneralPolygon(n_vertices, std::move(params), tok->loc);
        }
        else if (tok->token == "GeometricApproximation") {
          std::string_view type = dequote_string(*nextToken(TokenRequired));
          float m = parse_float(*nextToken(TokenRequired));

          target->GeometricApproximation(to_string_from_view(type), m, tok->loc);
        }
        else if (tok->token == "Geometry")
          basic_param_list_entrypoint(&Parser::Geometry, tok->loc);
        else
          syntax_error(*tok);
        break;

      // RI API
      case 'H':
        if (tok->token == "Hider") {
        }
        else if (tok->token == "Hyperboloid") {
        }
        else
          syntax_error(*tok);
        break;

      case 'I':
        if (tok->token == "Integrator")
          two_string_param_list_entrypoint(&Parser::Integrator, tok->loc);
        else if (tok->token == "Identity")
          target->Identity(tok->loc);
        // RI API
        else if (tok->token == "IfBegin") {
        }
        else if (tok->token == "IfEnd") {
        }
        else if (tok->token == "Illuminate") {
        }
        else if (tok->token == "Imager") {
        }
        else
          syntax_error(*tok);
        break;

      case 'L':
        if (tok->token == "LightSource")
          basic_param_list_entrypoint(&Parser::LightSource, tok->loc);
        // RI API
        else if (tok->token == "Light")
          two_string_param_list_entrypoint(&Parser::Light, tok->loc);
        else
          syntax_error(*tok);
        break;

      case 'M':
        if (tok->token == "MakeBrickMap") {
        }
        else if (tok->token == "MakeCubeFaceEnvironment") {
        }
        else if (tok->token == "MakeLatLongEnvironment") {
        }
        else if (tok->token == "MakeShadow") {
        }
        else if (tok->token == "MakeTexture") {
        }
        else if (tok->token == "Matte") {
        }
        else if (tok->token == "MotionBegin") {
        }
        else if (tok->token == "MotionEnd") {
        }
        else
          syntax_error(*tok);
        break;

      case 'N':
        if (tok->token == "NuPatch") {
        }
        else
          syntax_error(*tok);
        break;

      case 'O':
        if (tok->token == "ObjectBegin") {
          std::string_view n = dequote_string(*nextToken(TokenRequired));
          target->ObjectBegin(to_string_from_view(n), tok->loc);
        }
        else if (tok->token == "ObjectEnd")
          target->ObjectEnd(tok->loc);
        else if (tok->token == "ObjectInstance") {
          std::string_view n = dequote_string(*nextToken(TokenRequired));
          target->ObjectInstance(to_string_from_view(n), tok->loc);
        }
        else if (tok->token == "Option") {
          std::string_view n = dequote_string(*nextToken(TokenRequired));
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Option(to_string_from_view(n), std::move(params), tok->loc);
        }
        // RI API
        else if (tok->token == "Opacity") {
        }
        else if (tok->token == "Orientation") {
        }
        else
          syntax_error(*tok);
        break;

      case 'P':
        if (tok->token == "PixelFilter")
          basic_param_list_entrypoint(&Parser::PixelFilter, tok->loc);
        // RI API
        else if (tok->token == "Paraboloid") {
        }
        else if (tok->token == "Patch") {
        }
        else if (tok->token == "PatchMesh") {
        }
        else if (tok->token == "Pattern") {
          two_string_param_list_entrypoint(&Parser::Pattern, tok->loc);
        }
        else if (tok->token == "Perspective") {
        }
        else if (tok->token == "PixelSampleImager") {
        }
        else if (tok->token == "PixelVariance") {
        }
        else if (tok->token == "Points") {
        }
        else if (tok->token == "PointsGeneralPolygons") {
          std::vector<int> n_loops;
          parse_array_of_int(n_loops);

          std::vector<int> n_vertices;
          parse_array_of_int(n_vertices);

          std::vector<int> vertices;
          parse_array_of_int(vertices);

          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->PointsGeneralPolygons(
              n_loops, n_vertices, vertices, std::move(params), tok->loc);
        }
        else if (tok->token == "PointsPolygons") {
        }
        else if (tok->token == "Polygon") {
        }
        else if (tok->token == "ProcDelayedReadArchive") {
        }
        else if (tok->token == "Procedural") {
        }
        else if (tok->token == "Procedural2") {
        }
        else if (tok->token == "Projection")
          basic_param_list_entrypoint(&Parser::Projection, tok->loc);
        else
          syntax_error(*tok);
        break;

      // RI API
      case 'Q':
        if (tok->token == "Quantize") {
        }
        else
          syntax_error(*tok);
        break;

      case 'R':
        if (tok->token == "ReverseOrientation")
          target->ReverseOrientation(tok->loc);
        else if (tok->token == "Rotate") {
          float v[4];
          for (int i = 0; i < 4; ++i)
            v[i] = parse_float(*nextToken(TokenRequired));
          target->Rotate(v[0], v[1], v[2], v[3], tok->loc);
        }
        // RI API
        else if (tok->token == "ReadArchive") {
        }
        else if (tok->token == "RelativeDetail") {
        }
        else if (tok->token == "Resource") {
        }
        else if (tok->token == "ResourceBegin") {
        }
        else if (tok->token == "ResourceEnd") {
        }
        else
          syntax_error(*tok);
        break;

      case 'S':
        if (tok->token == "Scale") {
          float v[3];
          for (int i = 0; i < 3; ++i)
            v[i] = parse_float(*nextToken(TokenRequired));
          target->Scale(v[0], v[1], v[2], tok->loc);
        }
        // RI API
        else if (tok->token == "ScopedCoordinateSystem") {
        }
        else if (tok->token == "ScreenWindow") {
        }
        else if (tok->token == "ShadingInterpolation") {
        }
        else if (tok->token == "ShadingRate") {
        }
        else if (tok->token == "Shutter") {
        }
        else if (tok->token == "Sides") {
        }
        else if (tok->token == "Shader") {
        }
        else if (tok->token == "Skew") {
        }
        else if (tok->token == "SolidBegin") {
        }
        else if (tok->token == "SolidEnd") {
        }
        else if (tok->token == "Sphere") {
          bool bracketed = false;
          std::optional<Token> second = nextToken(TokenOptional);

          if (second.has_value()) {
            if (second->token == "[")
              bracketed = true;
            else
              unget(*second);
          }

          float m[4];
          for (int i = 0; i < 4; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));

          if (bracketed)
            if (nextToken(TokenRequired)->token != "]")
              syntax_error(*tok);
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });
          target->Sphere(m[0], m[1], m[2], m[3], std::move(params), tok->loc);
        }
        else if (tok->token == "SubdivisionMesh") {
        }
        else if (tok->token == "Surface") {
        }
        else if (tok->token == "System") {
        }
        else
          syntax_error(*tok);
        break;

      case 'T':
        if (tok->token == "TransformBegin") {
          {
            if (!warned_transform_begin_end_deprecated) {
              std::cout << &tok->loc << "TransformBegin/End are deprecated and should "
                        << "be replaced with AttributeBegin/End" << std::endl;
              warned_transform_begin_end_deprecated = true;
            }
            target->AttributeBegin(tok->loc);
          }
        }
        else if (tok->token == "TransformEnd") {
          target->AttributeEnd(tok->loc);
        }
        else if (tok->token == "Transform") {
          if (nextToken(TokenRequired)->token != "[")
            syntax_error(*tok);
          float m[16];
          for (int i = 0; i < 16; ++i)
            m[i] = parse_float(*nextToken(TokenRequired));
          if (nextToken(TokenRequired)->token != "]")
            syntax_error(*tok);
          target->transform(m, tok->loc);
        }
        else if (tok->token == "Translate") {
          float v[3];
          for (int i = 0; i < 3; ++i)
            v[i] = parse_float(*nextToken(TokenRequired));
          target->Translate(v[0], v[1], v[2], tok->loc);
        }
        else if (tok->token == "Texture") {
          std::string_view n = dequote_string(*nextToken(TokenRequired));
          std::string name = to_string_from_view(n);
          n = dequote_string(*nextToken(TokenRequired));
          std::string type = to_string_from_view(n);

          Token t = *nextToken(TokenRequired);
          std::string_view dequoted = dequote_string(t);
          std::string texName = to_string_from_view(dequoted);
          Parsed_Parameter_Vector params = parse_parameters(
              nextToken, unget, [&](const Token &t, const char *msg) {
                std::string token = to_string_from_view(t.token);
                std::string str = msg;
                parse_error(str.c_str(), &t.loc);
              });

          target->Texture(name, type, texName, std::move(params), tok->loc);
        }
        // RI API
        else if (tok->token == "TextureCoordinates") {
        }
        else if (tok->token == "Torus") {
        }
        else if (tok->token == "TransformPoints") {
        }
        else if (tok->token == "TrimCurve") {
        }
        else
          syntax_error(*tok);
        break;

      // RI API
      case 'v':
        if (tok->token == "version") {
          float version = parse_float(*nextToken(TokenRequired));
        }
        else
          syntax_error(*tok);
        break;

      case 'W':
        if (tok->token == "WorldBegin")
          target->WorldBegin(tok->loc);
        else if (tok->token == "WorldEnd")
          target->WorldEnd(tok->loc);
        else
          syntax_error(*tok);
        break;

      default:
        syntax_error(*tok);
    }
  }
}

void parse_files(Parser *target, std::vector<std::string> filenames)
{
  auto tok_error = [](const char *msg, const File_Loc *loc) {
    std::cerr << loc << " " << msg << std::endl;
    exit(-1);
  };

  // Process target description
  if (filenames.empty()) {
    // Parse target from standard input
    std::unique_ptr<Tokenizer> t = Tokenizer::create_from_file("-", tok_error);
    if (t)
      parse(target, std::move(t));
  }
  else {
    // Parse target from input files
    for (const std::string &fn : filenames) {
      if (fn != "-")
        set_search_directory(fn);

      std::unique_ptr<Tokenizer> t = Tokenizer::create_from_file(fn, tok_error);
      if (t)
        parse(target, std::move(t));
    }
  }

  target->end_of_files();
}

void parse_string(Parser *target, std::string str)
{
  auto tok_error = [](const char *msg, const File_Loc *loc) {
    std::cerr << loc << " " << msg << std::endl;
    exit(-1);
  };
  std::unique_ptr<Tokenizer> t = Tokenizer::create_from_string(std::move(str), tok_error);
  if (!t)
    return;
  parse(target, std::move(t));

  target->end_of_files();
}

CCL_NAMESPACE_END
