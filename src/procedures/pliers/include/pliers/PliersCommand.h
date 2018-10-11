/** @file PliersCommand.h
*    @brief Define PliersCommand class
*    
*    @author Alexis CARE
*/
#include "action/action_define.h"
#include "stdint.h"

/**
 * @defgroup Pliers The pliers action package
 * @{
 */

/**
 * @brief      Class for pliers command.
 */
class PliersCommand {
public:
  uint8_t action;
  uint8_t level;
  PliersCommand ();
  PliersCommand (uint8_t action, uint8_t level);
};
/**
 * @}
 */
