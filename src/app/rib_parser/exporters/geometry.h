
#ifndef __EXPORTERS_GEOMETRY_H__
#define __EXPORTERS_GEOMETRY_H__

#include "scene/scene.h"

#include "app/rib_parser/scene_entities.h"

CCL_NAMESPACE_BEGIN

void export_geometry(Scene *scene,
                     Instance_Scene_Entity &inst,
                     Instance_Definition_Scene_Entity *inst_def);

CCL_NAMESPACE_END
#endif  //__EXPORTERS_GEOMETRY_H__
