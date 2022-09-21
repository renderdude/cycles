#include "app/rib_parser/exporters/attribute.h"

#include "scene/attribute.h"
#include "scene/geometry.h"
#include "scene/scene.h"
#include "util/vector.h"

CCL_NAMESPACE_BEGIN

void apply_primvars(AttributeSet &attributes,
                    const ustring &name,
                    Parsed_Parameter *value,
                    AttributeElement elem,
                    AttributeStandard std)
{
  size_t size = 0;
  vector<float2> vf2;
  vector<float3> vf3;
  vector<float4> vf4;
  void *data = nullptr;

  TypeDesc attrType = TypeUnknown;
  if (value->type == "float") {
    if (value->name == "uv") {
      attrType = TypeFloat2;
      for (int i = 0; i < value->floats.size(); i += 2)
        vf2.push_back(make_float2(value->floats[i], value->floats[i + 1]));
      size = vf2.size() * sizeof(float2);
      data = vf2.data();
    }
    else {
      attrType = TypeFloat;
      size = value->floats.size() * sizeof(float);
      data = value->floats.data();
    }
  }
  else if (value->type == "color") {
    attrType = TypeVector;
    for (int i = 0; i < value->floats.size(); i += 3)
      vf3.push_back(make_float3(value->floats[i], value->floats[i + 1], value->floats[i + 2]));
    size = vf3.size() * sizeof(float3);
    data = vf3.data();
  }

  Attribute *const attr = attributes.add(name, attrType, elem);
  attr->std = std;

  assert(size == attr->buffer.size());
  std::memcpy(attr->data(), data, size);
}

CCL_NAMESPACE_END
