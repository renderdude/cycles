#ifndef ERROR_H
#define ERROR_H

#include <string>
#include <string_view>

CCL_NAMESPACE_BEGIN

struct File_Loc {
  File_Loc() = default;
  File_Loc(std::string_view filename) : filename(filename)
  {
  }
  std::string to_string() const
  {
    return "File_Loc";
  }

  std::string_view filename;
  int line = 1, column = 0;
};

CCL_NAMESPACE_END

#endif  // ERROR_H
