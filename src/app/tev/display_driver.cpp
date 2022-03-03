/*
 * Copyright 2011-2022 Blender Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "app/tev/display_driver.h"

#include "util/log.h"
#include "util/string.h"

CCL_NAMESPACE_BEGIN

/* --------------------------------------------------------------------
 * TEVDisplayDriver.
 */

TEVDisplayDriver::TEVDisplayDriver(const function<void()> &session_print):
  _session_print(session_print)
{
}

TEVDisplayDriver::~TEVDisplayDriver()
{
}

/* --------------------------------------------------------------------
 * Update procedure.
 */

void TEVDisplayDriver::update()
{
  _session_print();
}

void TEVDisplayDriver::next_tile_begin()
{
  /* Assuming no tiles used in interactive display. */
}

bool TEVDisplayDriver::update_begin(const Params &params, int texture_width, int texture_height)
{
  /* Note that it's the responsibility of TEVDisplayDriver to ensure updating and drawing
   * the texture does not happen at the same time. This is achieved indirectly.
   *
   * This locking is not performed on the Cycles side, because that would cause lock inversion. */

  /* Update texture dimensions if needed. */
  if (texture_.width != texture_width || texture_.height != texture_height) {
    texture_.width = texture_width;
    texture_.height = texture_height;

    if ( texture_.pixels )
      delete [] texture_.pixels;
    texture_.pixels = new half4[texture_width * texture_height * sizeof(half4)];
    /* Texture did change, and no pixel storage was provided. Tag for an explicit zeroing out to
     * avoid undefined content. */
    texture_.need_clear = true;
  }

  /* New content will be provided to the texture in one way or another, so mark this in a
   * centralized place. */
  texture_.need_update = true;

  return true;
}

void TEVDisplayDriver::update_end()
{
}

/* --------------------------------------------------------------------
 * Texture buffer mapping.
 */

half4 *TEVDisplayDriver::map_texture_buffer()
{
  half4 *mapped_rgba_pixels = texture_.pixels;
  if (!mapped_rgba_pixels) {
    LOG(ERROR) << "Error mapping TEVDisplayDriver pixel buffer object.";
  }

  if (texture_.need_clear) {
    const int64_t texture_width = texture_.width;
    const int64_t texture_height = texture_.height;
    memset(reinterpret_cast<void *>(mapped_rgba_pixels),
           0,
           texture_width * texture_height * sizeof(half4));
    texture_.need_clear = false;
  }

  return mapped_rgba_pixels;
}

void TEVDisplayDriver::unmap_texture_buffer()
{
}

/* --------------------------------------------------------------------
 * Drawing.
 */

void TEVDisplayDriver::clear()
{
  texture_.need_clear = true;
}

void TEVDisplayDriver::draw(const Params &params)
{
  /* See do_update_begin() for why no locking is required here. */
  if (texture_.need_clear) {
    /* Texture is requested to be cleared and was not yet cleared.
     * Do early return which should be equivalent of drawing all-zero texture. */
    return;
  }
}

CCL_NAMESPACE_END
