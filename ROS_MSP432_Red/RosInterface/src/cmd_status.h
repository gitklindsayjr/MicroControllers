#ifndef CMD_STATUS_H
#define CMD_STATUS_H
/****************************************************************************
 *  Copyright (C) 2017 Ken Lindsay
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************
 *  DESCRIPTION
 * Serial interface command and interface protocall.  Commands and returned status
 * across a serial interface are tokenized using 2 character ASCII.  The command/start
 * strings are delimited by "<" less than and ">" greater than characters.
  *****************************************************************************/
namespace cmdStatus
{
#define START_DELIMITER             "<"
#define END_DELIMITER               ">"
#define END_DELIMITER_NL            ">\n"

// Default: commands
#define NO_COMMAND                   0
#define RESTART_CMD                  (('R'<<8)|'S')
#define QUIT_CMD                     (('Q'<<8)|'U')
#define ACKNOWLEDGE                  (('A'<<8)|'K')

// Default: command Strings
#define RESTART_CMD_                 "<RS>\n"
#define QUIT_CMD_STR                 "<QU>\n"
#define ACKNOWLEDGE_STR              "<AK>\n"

// Todo: Add commands here, these are just examples
#define RED_LED_TOGGLE_CMD           (('L'<<8)|'T')
#define RED_LED_ON_CMD               (('L'<<8)|'1')
#define RED_LED_OFF_CMD              (('L'<<8)|'0')
#define SET_STRING_CMD               (('S'<<8)|'S')
#define SET_INTEGER_CMD              (('S'<<8)|'I')
#define SET_FLOAT_CMD                (('S'<<8)|'F')
#define GET_STRING_CMD               (('G'<<8)|'S')
#define GET_INTEGER_CMD              (('G'<<8)|'I')
#define GET_FLOAT_CMD                (('G'<<8)|'F')


// Todo: Add command strings here, these are just examples
#define RED_LED_TOGGLE_STR            "LT"
#define RED_LED_ON_STR                "L1"
#define RED_LED_OFF_STR               "L0"
#define SET_STRING_CMD_STR            "SS"
#define SET_INTEGER_CMD_STR           "SI"
#define SET_FLOAT_CMD_STR             "SF"
#define GET_STRING_CMD_STR            "GS"
#define GET_INTEGER_CMD_STR           "GI"
#define GET_FLOAT_CMD_STR             "GF"

// Todo: Add transmit data responses tokens, these are just examples
#define RED_LED_TOGGLE_MSG             (('l'<<8)|'t')
#define STRING_TYPE_MESSAGE            (('s'<<8)|'t')
#define INTEGER_TYPE_MESSAGE           (('i'<<8)|'n')
#define FLOAT_TYPE_MESSAGE             (('f'<<8)|'l')

// Todo: Add transmit data response token string, these are just examples
#define RED_LED_TOGGLE_MSG_STR         "lt"
#define STRING_TYPE_MESSAGE_STR        "st"
#define INTEGER_TYPE_MESSAGE_STR       "in"
#define FLOAT_TYPE_MESSAGE_STR         "fl"

enum _errors { NO_ERROR, MESSAGE_ERROR, MESSAGE_LOOP_PARSE_OVERRUN, MESSAGE_PARSE_DELIMIT };
}
#endif /* CMD_STATUS_H_ */

