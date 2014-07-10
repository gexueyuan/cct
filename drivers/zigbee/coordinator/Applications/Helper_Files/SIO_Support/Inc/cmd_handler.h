/**
 * @file sio_handler.h
 *
 * @brief This file contains macros and function prototypes for SIO handling.
 *
 * $Id: cmd_handler.h 29087 2011-11-02 14:59:39Z yogesh.bellan $
 *
 * @author    Atmel Corporation: http://www.atmel.com
 * @author    Support email: avr@atmel.com
 */
/*
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

/* === Includes ============================================================= */


/* === Macros =============================================================== */


/* === Types ================================================================ */


/* === Externals ============================================================ */


/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

    command_result_t mac_reset_handler(command_table_t *table_entry,
                                       char **cmd_line,
                                       cmd_response_cb_t response_cb);

    command_result_t auto_addr_alloc_handler(command_table_t *table_entry,
                                             char **cmd_line,
                                             cmd_response_cb_t response_cb);

    command_result_t mac_pib_handler(command_table_t *table_entry,
                                     char **cmd_line,
                                     cmd_response_cb_t response_cb);

    command_result_t mac_assoc_handler (command_table_t *table_entry,
                                        char **cmd_line,
                                        cmd_response_cb_t response_cb);

    command_result_t mac_start_handler (command_table_t *table_entry,
                                        char **cmd_line,
                                        cmd_response_cb_t response_cb);

    command_result_t mac_assoc_rsp_handler (command_table_t *table_entry,
                                            char **cmd_line,
                                            cmd_response_cb_t response_cb);

    command_result_t mac_data_req_handler(command_table_t *table_entry,
                                          char **cmd_line,
                                          cmd_response_cb_t response_cb);

    command_result_t mac_purge_req_handler(command_table_t *table_entry,
                                           char **cmd_line,
                                           cmd_response_cb_t response_cb);

    command_result_t mac_scan_req_handler (command_table_t *table_entry,
                                           char **cmd_line,
                                           cmd_response_cb_t response_cb);

    command_result_t mac_disassoc_handler (command_table_t *table_entry,
                                           char **cmd_line,
                                           cmd_response_cb_t response_cb);

    command_result_t mac_poll_req_handler(command_table_t *table_entry,
                                          char **cmd_line,
                                          cmd_response_cb_t response_cb);

    command_result_t mac_synq_req_handler(command_table_t *table_entry,
                                          char **cmd_line,
                                          cmd_response_cb_t response_cb);

    command_result_t mac_rx_enable_req_handler(command_table_t *table_entry,
                                               char **cmd_line,
                                               cmd_response_cb_t response_cb);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* CMD_HANDLER_H */

/* EOF */
