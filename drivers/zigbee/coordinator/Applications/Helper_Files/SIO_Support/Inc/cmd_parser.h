/**
 * @file sio_handler.h
 *
 * @brief This file contains macros and function prototypes for SIO handling.
 *
 * $Id: cmd_parser.h 29087 2011-11-02 14:59:39Z yogesh.bellan $
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
#ifndef CMD_PARSER_H
#define CMD_PARSER_H

/* === Includes ============================================================= */


/* === Macros =============================================================== */


/* === Types ================================================================ */

/**
 * Command result enum
 */
typedef enum command_result_tag
{
    /** Command execution is complete and has been successful */
    CMD_RESULT_OK = 0,
    /** The entire command line has been executed successfully */
    CMD_RESULT_FINISHED = 1,
    /** Command execution has run into an error */
    CMD_RESULT_ERROR = 4,
    /** Command execution still in progress or this is an unsolicited response */
    CMD_RESULT_PENDING = 2
} command_result_t;

/**
 * Command type enum
 */
typedef enum command_type_tag
{
    CMD_TYPE_EXEC = 0x01,
    CMD_TYPE_SET = 0x02,
    CMD_TYPE_GET = 0x04,
    CMD_TYPE_TEST = 0x08,
    /** Command only supports setting/getting a value (eg. S-register cmds)*/
    CMD_TYPE_VALUE = CMD_TYPE_SET | CMD_TYPE_GET | CMD_TYPE_TEST,
    /** Command supports set/get and execute (eg. most basic commands)*/
    CMD_TYPE_ALL = CMD_TYPE_VALUE | CMD_TYPE_EXEC
} command_type_t;

/**
 * Command response callback function
 */
typedef void (*cmd_response_cb_t) (command_result_t result, char *rsp_str);

/**
 * Command table type
 */
typedef struct command_table_tag
{
    /**
     * String containing the command which may be interpreted variously
     * depending upon the kind of table: basic, extended or S-register.
     */
    char *cmd;

    /** Type of the command - indicates which of GET/SET/EXECUTE is supported */
    command_type_t type;

    /** Pointer to the value, if any, that can be set with the command */
    uint16_t *value;

    /** Minimum legal value that can be stored */
    uint8_t min;

    /** Maximum value that can be stored */
    uint16_t max;

    /** Handler for the command (executed in case of SET/EXEC commands */
    command_result_t (*handler)(struct command_table_tag *table_entry,
                                char **cmd_line,
                                cmd_response_cb_t response_cb);
} command_table_t;

/* === Externals ============================================================ */


/* === Prototypes =========================================================== */

#ifdef __cplusplus
extern "C" {
#endif

    command_result_t execute_ext_cmd(char **cmd_line, cmd_response_cb_t response_cb);

    command_result_t execute_from_table(command_table_t *cmd_table,
                                        char **cmd_line,
                                        cmd_response_cb_t response_cb);

    int get_number (char **cmd_line);
    int get_hex_number (char **cmd_line);
    uint8_t get_hex_byte (char **cmd_line);
    uint8_t get_next_arg (char **cmd_line);

    command_type_t get_cmd_type (char **cmd_line);

    void send_response(command_result_t result, char *rsp_str);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* CMD_PARSER_H */

/* EOF */
