#ifndef NWK_MSG_TYPES_H
#define NWK_MSG_TYPES_H

/* === Includes ============================================================= */

#include <stdbool.h>
#include "tal.h"
#include "qmm.h"
#include "mac_build_config.h"
#ifdef ENABLE_RTB
#include "rtb.h"
#endif  /* ENABLE_RTB */

#include "nwk_msg_const.h"
/* === Macros =============================================================== */


/* === Types ================================================================ */

typedef struct nlme_reset_req_tag
{
    /**< This identifies the message as \ref MLME_RESET_REQUEST */
    enum nwk_msg_code cmdcode;
    /**
     * If TRUE, the MAC sublayer is reset and all MAC PIB attributes are set to
     * their default values. If FALSE, the MAC sublayer is reset but all MAC PIB
     * attributes retain their values prior to the generation of the
     * MLME-RESET.request primitive.
     */
    uint8_t WarmStart;
} nlme_reset_req_t;

typedef struct nlme_reset_conf_tag
{
    /**< This identifies the message as \ref MLME_RESET_CONFIRM */
    enum nwk_msg_code cmdcode;
    /** The result of the reset operation. */
    uint8_t WarmStart;
    uint8_t status;
} nlme_reset_conf_t;
/* === Externals ============================================================ */



/* === Prototypes =========================================================== */




#endif

