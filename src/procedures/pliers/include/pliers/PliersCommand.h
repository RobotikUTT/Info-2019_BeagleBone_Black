// #include ".h"
#include "action/action_define.h"
#include "stdint.h"


class PliersCommand {
public:
  uint8_t action;
  uint8_t level;
  PliersCommand ();
  PliersCommand (uint8_t action, uint8_t level);
};
