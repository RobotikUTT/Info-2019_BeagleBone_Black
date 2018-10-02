/** @file PliersCommand.h
*    @brief Pliers commande class
*    
*/
#include "pliers/PliersCommand.h"


/**
 * @brief      Constructs the object.
 * 
 * @details 	Action type:
 * 				- TAKE_BLOCK 1
 *				- RELEASE_BLOCK 0
 *				- SET_PLIERS 2
 *
 * @param[in]  act   The action type
 * @param[in]  lvl   The level of the pliers
 */
PliersCommand::PliersCommand(uint8_t act, uint8_t lvl):action(act),level(lvl){
}

/**
 * @brief      Constructs the object.
 */
PliersCommand::PliersCommand():action(0),level(0){
}
