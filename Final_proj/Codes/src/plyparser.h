#ifndef CGL_PLYPARSER_H
#define CGL_PLYPARSER_H

#include "rply-1.1.4/rply.h"
#include <stdio.h>

namespace CGL {

  static int face_cb(p_ply_argument argument);
  static int vertex_cb(p_ply_argument argument);

} // namespace CGL



#endif
