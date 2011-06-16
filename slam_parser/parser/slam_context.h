#ifndef SLAM_CONTEXT_H
#define SLAM_CONTEXT_H

#include "commands.h"
#include <vector>

namespace SlamParser {

class SlamContext
{
  public:
    SlamContext();
    virtual ~SlamContext();

    virtual bool process(CommandNode* commandNode);
};

} // end namespace

#endif
