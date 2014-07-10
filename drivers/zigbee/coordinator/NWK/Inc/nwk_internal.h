#ifndef NWK_INTERNAL_H
#define NWK_INTERNAL_H
/* === Includes ============================================================= */

#include "pal.h"


/* === Macros =============================================================== */


/* === Types ================================================================ */



/* === Externals ============================================================ */
//extern bool nwk_busy;

/* === Prototypes =========================================================== */

void nwk_dispatch_event(uint8_t *event);
void dispatch_event(uint8_t *event);
uint8_t nwk_get_ib_attribute_size(NWK_NibId_t pib_attribute_id);


#endif
