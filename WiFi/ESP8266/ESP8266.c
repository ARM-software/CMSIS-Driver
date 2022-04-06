/* -----------------------------------------------------------------------------
 * Copyright (c) 2019-2022 Arm Limited (or its affiliates). All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        23. March 2022
 *
 * Project:      ESP8266 WiFi Driver
 * -------------------------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "ESP8266.h"
#include "ESP8266_Serial.h"

#include "WiFi_ESP8266_Os.h"

/* Control block */
static AT_PARSER_HANDLE AT_Cb;

/* Pointer to parser control block */
#define pCb     (&AT_Cb)

/* Pointer to parser buffer memory */
#define pMem    (&AT_Cb.mem)

/* String list definition */
typedef const struct  {
  const char *str;
} STRING_LIST_t;


/* Static functions */
static int32_t     ReceiveData (void);
static uint8_t     AnalyzeLineData (void);
static uint8_t     GetCommandCode       (BUF_LIST *mem);
static uint8_t     GetASCIIResponseCode (BUF_LIST *mem);
static uint8_t     GetGMRResponseCode   (BUF_LIST *mem);
static uint8_t     GetCtrlResponseCode  (BUF_LIST *mem);
static int32_t     GetRespArg (uint8_t *buf, uint32_t sz);
static int32_t     CmdOpen   (uint8_t cmd_code, uint32_t cmd_mode, char *buf);
static int32_t     CmdSend   (uint8_t cmd, char *buf, int32_t num);
static const char *CmdString (uint8_t cmd);
static int32_t     CmdSetWFE (uint8_t cmd);
static void        AT_Parse_IP  (char *buf, uint8_t ip[]);
static void        AT_Parse_MAC (char *buf, uint8_t mac[]);


/* Command list (see also CommandCode_t) */
static STRING_LIST_t List_PlusResp[] = {
  { "IPD"              },
  { "CWLAP"            },
  { "CWJAP_CUR"        },
  { "CWQAP"            },
  { "CWSAP_CUR"        },
  { "CWMODE_CUR"       },
  { "CWHOSTNAME"       },
  { "CIPSTAMAC_CUR"    },
  { "CIPAPMAC_CUR"     },
  { "RFPOWER"          },
  { "CIPSTA_CUR"       },
  { "CIPAP_CUR"        },
  { "CIPDNS_CUR"       },
  { "CWDHCP_CUR"       },
  { "CWDHCPS_CUR"      },
  { "CWAUTOCONN"       },
  { "CWLIF"            },
  { "UART_CUR"         },
#if (AT_VARIANT == AT_VARIANT_ESP)
#if (AT_VERSION >= AT_VERSION_1_6_0_0) && (AT_VERSION < AT_VERSION_1_7_0_0)
  { "SYSMSG"           },
#elif (AT_VERSION >= AT_VERSION_1_7_0_0)
  { "SYSMSG_CUR"       },
#endif
#else
  { "SYSMSG_CUR"       },
#endif
  { "CIPSTATUS"        },
  { "CIPDOMAIN"        },
  { "CIPSTART"         },
  { "CIPCLOSE"         },
  { "PING"             },
  { "CIPSEND"          },
  { "CIPMUX"           },
  { "CIPSERVER"        },
  { "CIPSERVERMAXCONN" },
  { "RST"              },
  { "GMR"              },
  { "LINK_CONN"        },
  { "STA_CONNECTED"    },
  { "STA_DISCONNECTED" },
  { "SLEEP"            },
  { "E"                },
  { ""                 }
};

/* Command codes */
typedef enum {
  CMD_IPD         = 0,
  CMD_CWLAP,
  CMD_CWJAP_CUR,
  CMD_CWQAP,
  CMD_CWSAP_CUR,
  CMD_CWMODE_CUR,
  CMD_CWHOSTNAME,
  CMD_CIPSTAMAC_CUR,
  CMD_CIPAPMAC_CUR,
  CMD_RFPOWER,
  CMD_CIPSTA_CUR,
  CMD_CIPAP_CUR,
  CMD_CIPDNS_CUR,
  CMD_CWDHCP_CUR,
  CMD_CWDHCPS_CUR,
  CMD_CWAUTOCONN,
  CMD_CWLIF,
  CMD_UART_CUR,
  CMD_SYSMSG_CUR,
  CMD_CIPSTATUS,
  CMD_CIPDOMAIN,
  CMD_CIPSTART,
  CMD_CIPCLOSE,
  CMD_PING,
  CMD_CIPSEND,
  CMD_CIPMUX,
  CMD_CIPSERVER,
  CMD_CIPSERVERMAXCONN,
  CMD_RST,
  CMD_GMR,
  CMD_LINK_CONN,
  CMD_STA_CONNECTED,
  CMD_STA_DISCONNECTED,
  CMD_SLEEP,
  CMD_ECHO        = 0xFD, /* Command Echo                 */
  CMD_TEST        = 0xFE, /* AT startup (empty command)   */
  CMD_UNKNOWN     = 0xFF  /* Unknown or unhandled command */
} CommandCode_t;


/* Generic responses (see AT_RESP_x definitions) */
static STRING_LIST_t List_ASCIIResp[] = {
  { "OK"                },
  { "ERROR"             },
  { "FAIL"              },
  { "SEND OK"           },
  { "SEND FAIL"         },
  { "busy p..."         },
  { "busy s..."         },
  { "ALREADY CONNECTED" },
/* Generic responses redirected to AT_Notify function */
  { "WIFI CONNECTED"    },
  { "WIFI GOT IP"       },
  { "WIFI DISCONNECT"   },
/* Ignored */
#if 0
  { "no change"         },
  { "DNS Fail"          },
  { "UNLINK"            },
#endif
/* Special responses */
  { "AT"                },
  { "ready"             },
  { "ERR CODE"          },
};


/* List of strings received in response to AT+GMR */
static STRING_LIST_t List_Gmr[] = {
  { "AT version"   },
  { "SDK version"  },
  { "compile time" },
  { "Bin version"  }
};

/* GMR codes */
#define AT_GMR_AT_VER           0
#define AT_GMR_SDK_VER          1
#define AT_GMR_COMP_TIME        2
#define AT_GMR_BIN_VER          3
#define AT_GMR_UNKNOWN          0xFF


/* Control strings */
static STRING_LIST_t List_Ctrl[] = {
  { "CONNECT" },
  { "CLOSED"  }
};

/* Control codes */
#define AT_CTRL_CONNECT        0
#define AT_CTRL_CLOSED         1
#define AT_CTRL_UNKNOWN        0xFF


/* ------------------------------------------------------------------------- */

/* Get pointer to command string */
static const char *CmdString (uint8_t cmd) {
  return (List_PlusResp[cmd].str);
}

/* ------------------------------------------------------------------------- */

/**
  Serial callback.
*/
void Serial_Cb (uint32_t cb_event) {

  if (cb_event & SERIAL_CB_TX_DATA_COMPLETED) {
    /* Serial transmit completed */
    AT_Notify (AT_NOTIFY_TX_DONE, NULL);
  }

  if (cb_event & SERIAL_CB_RX_DATA_AVAILABLE) {
    /* Serial received data */
    AT_Notify (AT_NOTIFY_EXECUTE, NULL);
  }
}

/* ------------------------------------------------------------------------- */

/**
  Initialize parser.
  
  \return 0: initialized, pooling is required
          1: initialized, event driven operation
   negative: initialization error
*/
int32_t AT_Parser_Initialize (void) {
  osMemoryPoolAttr_t mp_attr;
  osMemoryPoolId_t   mp_id;
  int32_t ex, stat;

  stat = -1;

  mp_attr = AT_Parser_MemPool_Attr;
  mp_id = osMemoryPoolNew (PARSER_BUFFER_BLOCK_COUNT, PARSER_BUFFER_BLOCK_SIZE, &mp_attr);

  if (mp_id != NULL) {
    /* Init serial interface */
    ex = Serial_Initialize ();

    if (ex == 0) {
      /* Initialized, pooling mode */
      stat = 0;
    }
    else {
      if (ex == 1) {
        /* Initialized, rx timeout signal event driven */
        stat = 1;
      }
    }
  }

  if (stat >= 0) {
    /* Setup memory pool */
    BufInit (mp_id, NULL, &pCb->mem);
    BufInit (mp_id, NULL, &pCb->resp);

    /* Set initial state */
    pCb->state     = AT_STATE_ANALYZE;
    pCb->cmd_sent  = CMD_UNKNOWN;
    pCb->gen_resp  = 0U;
    pCb->msg_code  = 0U;
    pCb->resp_code = CMD_UNKNOWN;
    pCb->resp_len  = 0U;
  }

  if (stat < 0) {
    /* Clean up resources */
    Serial_Uninitialize();

    if (mp_id != NULL) {
      osMemoryPoolDelete (mp_id);
    }
  }

  return (stat);
}

/**
  Uninitialize parser.
*/
int32_t AT_Parser_Uninitialize (void) {

  Serial_Uninitialize();

  BufUninit(pMem);

  osMemoryPoolDelete (pMem->mp_id);

  pCb->mem.mp_id  = NULL;
  pCb->resp.mp_id = NULL;

  return (0);
}

/**
  Get current serial interface mode and baud rate.
*/
int32_t AT_Parser_GetSerialCfg (AT_PARSER_COM_SERIAL *info) {
  return (Serial_GetMode ((SERIAL_MODE *)info));
}

/**
  Set serial interface mode and baud rate.
*/
int32_t AT_Parser_SetSerialCfg (AT_PARSER_COM_SERIAL *info) {
  return (Serial_SetMode ((SERIAL_MODE *)info));
}

/**
  Reset parser.
*/
void AT_Parser_Reset (void) {

  /* Flush parser buffer */
  BufFlush (0, pMem);

  /* Reset state */
  pCb->state     = AT_STATE_ANALYZE;
  pCb->cmd_sent  = CMD_UNKNOWN;
  pCb->gen_resp  = 0U;
  pCb->msg_code  = 0U;
  pCb->resp_code = CMD_UNKNOWN;
  pCb->resp_len  = 0U;
}

/**
  Execute AT command parser.
*/
void AT_Parser_Execute (void) {
  uint8_t crlf[] = {'\r', '\n'};
  int32_t n;
  uint32_t sleep;
  uint32_t p;

  sleep = 0U;

  while (sleep == 0) {

    /* Receive serial data */
    n = ReceiveData();

    if (n == 1U) {
      /* Out of memory */
      AT_Notify (AT_NOTIFY_OUT_OF_MEMORY, pCb->mem.mp_id);
    }

    switch (pCb->state) {
      case AT_STATE_ANALYZE:
        pCb->state = AnalyzeLineData();
        break;

      case AT_STATE_WAIT:
        /* Not enough data in buffer to complete operation */
        sleep = 1U;

        /* Next state */
        pCb->state = AT_STATE_ANALYZE;
        break;

      case AT_STATE_FLUSH:
        /* Flush current response till first CRLF */
        n = BufFind (crlf, 2, pMem);

        if (n != -1) {
          /* Flush buffer including crlf */
          BufFlush ((uint32_t)n + 2, pMem);
        }

        /* Start analyzing again */
        pCb->state = AT_STATE_ANALYZE;
        break;

      case AT_STATE_RECV_DATA:
        /* Copy IPD data */
        /* Set pointer to source memory buffer */
        p = (uint32_t)pMem;

        /* Call notify using pointer to memory buffer */
        AT_Notify (AT_NOTIFY_CONNECTION_RX_DATA, &p);

        /* On return, p must contain number of bytes left to receive */
        if (p == 0) {
          /* Packet is received */
          sleep = 1U;

          /* Next state */
          pCb->state = AT_STATE_ANALYZE;
        }
        else {
          if (p == pCb->ipd_rx) {
            /* Application did not read anything */
            sleep = 1U;
          }
          pCb->ipd_rx = p;
        }
        break;

      case AT_STATE_RESP_DATA:
        /* Received +CMD response */
        if (pCb->resp_code == CMD_IPD) {
          /* Copy response (including ':' character) */
          BufCopy (&(pCb->resp), &(pCb->mem), pCb->resp_len+1);

          /* Receive network data (+IPD) */
          pCb->ipd_rx = 0U;

          AT_Notify (AT_NOTIFY_CONNECTION_RX_INIT, &(pCb->ipd_rx));

          if (pCb->ipd_rx == 0U) {
            /* Socket is out of memory */
            AT_Notify (AT_NOTIFY_OUT_OF_MEMORY, NULL);
          }

          /* Start receiving data */
          pCb->state = AT_STATE_RECV_DATA;
        }
        else {
          /* Response data arrived */
          if (pCb->resp_code == CMD_PING) {
            /* Artificially add '+PING:' string */
            BufWrite ((uint8_t *)"+PING:", 6, &(pCb->resp));
            /* Flush '+' from the original response */
            BufFlushByte (&(pCb->mem));
            /* Adjust response length for the flushed byte */
            pCb->resp_len -= 1U;
          }

          /* Copy response (including "\r\n" characters) */
          BufCopy (&(pCb->resp), &(pCb->mem), pCb->resp_len+2);

          pCb->state = AT_STATE_ANALYZE;

          if (pCb->resp_code == CMD_LINK_CONN) {
            /* Connection established (+LINK_CONN) */
            AT_Notify (AT_NOTIFY_CONNECTION_OPEN, NULL);
          }
          else if (pCb->resp_code == CMD_STA_CONNECTED) {
            /* Station connected to local AP (+STA_CONNECTED:<sta_mac>) */
            AT_Notify (AT_NOTIFY_STATION_CONNECTED, NULL);
          }
          else if (pCb->resp_code == CMD_STA_DISCONNECTED) {
            /* Station disconnected from local AP (+STA_DISCONNECTED:<sta_mac>) */
            AT_Notify (AT_NOTIFY_STATION_DISCONNECTED, NULL);
          }
          else if (pCb->resp_code != CMD_UNKNOWN) {
            /* Command response (+XXX in buffer) */
            pCb->state = AT_STATE_ANALYZE;
          }
          else {
            /* Response unknown/unhandled */
            pCb->state = AT_STATE_FLUSH;
          }
        }
        break;

      case AT_STATE_RESP_GMR:
        /* +GMR: copy response into response buffer */
        BufCopy (&(pCb->resp), &(pCb->mem), pCb->resp_len+2);

        pCb->state = AT_STATE_ANALYZE;
        break;

      case AT_STATE_RESP_GEN:
        /* Generic response received */
        switch (pCb->msg_code) {
          case AT_RESP_OK:
          case AT_RESP_ERROR:
          case AT_RESP_ALREADY_CONNECTED:
          case AT_RESP_SEND_OK:
          case AT_RESP_SEND_FAIL:
            /* Set generic command response */
            pCb->gen_resp = pCb->msg_code;

            /* Application waits for response */
            AT_Notify (AT_NOTIFY_RESPONSE_GENERIC, NULL);

            sleep = 1U;
            break;

          case AT_RESP_BUSY_P:
          case AT_RESP_BUSY_S:
            /* Busy processing or busy sending */
            break;

          case AT_RESP_WIFI_CONNECTED:
            AT_Notify (AT_NOTIFY_CONNECTED, NULL);
            break;

          case AT_RESP_WIFI_GOT_IP:
            AT_Notify (AT_NOTIFY_GOT_IP, NULL);
            break;

          case AT_RESP_WIFI_DISCONNECT:
            AT_Notify (AT_NOTIFY_DISCONNECTED, NULL);
            break;

          case AT_RESP_READY:
            pCb->gen_resp = pCb->msg_code;

            AT_Notify (AT_NOTIFY_READY, NULL);
            sleep = 1U;
            break;
          
          case AT_RESP_ERR_CODE:
            /* Error code received */
            /* Artificially add '+' character and copy response */
            BufWriteByte ('+', &(pCb->resp));
            BufCopy (&(pCb->resp), &(pCb->mem), pCb->resp_len+2);

            AT_Notify (AT_NOTIFY_ERR_CODE, NULL);
            break;
          
          default:
          case AT_RESP_UNKNOWN:
            /* Unknown response */
            break;
        }

        /* Set next state */
        pCb->state = AT_STATE_FLUSH;
        break;

      case AT_STATE_SEND_DATA:
        /* Received '>' character */
        AT_Notify (AT_NOTIFY_REQUEST_TO_SEND, NULL);

        sleep = 1U;

        /* Next state */
        pCb->state = AT_STATE_FLUSH;
        break;

      case AT_STATE_RESP_CTRL:
        /* Control code arrived */
        if (pCb->ctrl_code == AT_CTRL_CONNECT) {
          /* <conn_id>,CONNECT */
          AT_Notify (AT_NOTIFY_CONNECTION_OPEN, NULL);
        }
        else if (pCb->ctrl_code == AT_CTRL_CLOSED) {
          /* <conn_id>,CLOSED */
          AT_Notify (AT_NOTIFY_CONNECTION_CLOSED, NULL);
        }

        /* Next state */
        pCb->state = AT_STATE_FLUSH;
        break;

      case AT_STATE_RESP_ECHO:
        /* Command echo received */
        /* Next state */
        pCb->state = AT_STATE_FLUSH;
        break;
    }
  }
}


/*
  Retrieve data from the serial interface and copy the data into the buffer.
*/
static int32_t ReceiveData (void) {
  static uint32_t sz_buf;
  static uint32_t n_prev;
  BUF_MEM *buf;
  uint32_t n, cnt, num;
  int32_t err;
  

  if (sz_buf == 0) {
    sz_buf = BufGetSize(pMem);
  }

  err = 0;
  num = 0U;
  n = Serial_GetRxCount();

  if (n == n_prev) {
    /* No new bytes received */
    n = 0U;
  }

  while (num < n) {
    /* Determine free space in the buffer */
    buf = BufGetTail (pMem);

    if (buf != NULL) {
      cnt = sz_buf - buf->wr_idx;
    } else {
      cnt = 0U;
    }

    if (cnt != 0U) {
      /* We can read cnt bytes in one pass */

      if (n < cnt) {
        /* Number of bytes received is less than we can read */
        cnt = n;
      }

      /* Read actual data */
      cnt = (uint32_t)Serial_ReadBuf (&buf->data[buf->wr_idx], cnt);

      if (cnt != 0) {
        buf->wr_idx += cnt;
        num         += cnt;
      } else {
        /* Serial buffer empty? */
        err = 2U;
      }
    }
    else {
      /* Out of memory */
      err = 1U;
    }

    if (err != 0U) {
      break;
    }
  }

  return (err);
}

#define AT_LINE_NODATA       (1U << 0) /* Line is empty                        */
#define AT_LINE_INCOMPLETE   (1U << 1) /* Line contains incomplete response    */
#define AT_LINE_PLUS         (1U << 2) /* Line starts with '+' response        */
#define AT_LINE_COLON        (1U << 3) /* Line contains ':' character          */
#define AT_LINE_ASCII        (1U << 4) /* Line starts with ASCII characters    */
#define AT_LINE_CRLF         (1U << 5) /* Line contains CRLF characters        */
#define AT_LINE_TXREQ        (1U << 6) /* Line starts with '>' character       */
#define AT_LINE_CTRL         (1U << 7) /* Line starts with numeric character   */
#define AT_LINE_NUMBER       (1U << 8) /* Line contains numeric character      */

/**
  Analyze received data and set AT_LINE_n flags based on the line content.

  \return AT_LINE flags
*/
static uint32_t AnalyzeLine (BUF_LIST *mem) {
  uint8_t  crlf[] = {'\r', '\n'};
  uint8_t  b;       /* Received byte */
  uint32_t flags;   /* Analysis flags */
  int32_t  val;

  flags = 0U;

  do {
    /* Peek current byte from list buffer */
    val = BufPeekByte (mem);

    if (val < 0) {
      /* Buffer empty */
      flags |= AT_LINE_NODATA;
      break;
    }

    b = (uint8_t)val;

    if (b == '+') {
      /* Found: +command response */
      flags |= AT_LINE_PLUS;

      /* Check if colon is received */
      val = BufFindByte (':', mem);

      if (val != -1) {
        flags |= AT_LINE_COLON;
      } else {
        /* Not terminated */
        flags |= AT_LINE_INCOMPLETE;

        /* Check if next character is a number (PING response) */
        val = BufPeekOffs(1, mem);

        if (val != -1) {
          b = (uint8_t)val;

          if ((b >= '0') && (b <= '9')) {
            flags &= ~AT_LINE_INCOMPLETE;
            flags |=  AT_LINE_NUMBER;
          }
        }
      }
    }
    else if (b == '>') {
      /* Found: data input request */
      flags |= AT_LINE_TXREQ;
    }
    else if (((b >= 'A') && (b <= 'Z')) || ((b >= 'a') && (b <= 'z'))) {
      /* Found: command ASCII response */
      flags |= AT_LINE_ASCII;

      /* Check if terminated */
      val = BufFind (crlf, 2, mem);

      if (val != -1) {
        pCb->resp_len = (uint8_t)val;

        flags |= AT_LINE_CRLF;
      } else {
        /* Not terminated */
        flags |= AT_LINE_INCOMPLETE;
      }
    }
    else if ((b >= '0') && (b <= '9')) {
      /* [<link ID>,] CLOSED response ? */
      flags |= AT_LINE_CTRL;

      /* Check if terminated */
      val = BufFind (crlf, 2, mem);

      if (val != -1) {
        pCb->resp_len = (uint8_t)val;

        flags |= AT_LINE_CRLF;
      } else {
        /* Not terminated */
        flags |= AT_LINE_INCOMPLETE;
      }
    }
    else {
      /* Unknown character, flush it and continue */
      BufFlushByte(mem);
    }
  } while (flags == 0U);

  /* Return analysis result */
  return (flags);
}

/* -------------------------------------------------------------------- */
/**
  Analyze the received content and decide what to do with it.

  \return next parser state, see AT_STATE_ definitions.
*/
static uint8_t AnalyzeLineData (void) {
  uint8_t  crlf[] = {'\r', '\n'};
  uint8_t  code;
  uint8_t  rval;
  int32_t  n;
  uint32_t flags;

  flags = AnalyzeLine(pMem);

  if ((flags & AT_LINE_NODATA) || (flags & AT_LINE_INCOMPLETE)) {
    /* No data or incomplete response */
    rval = AT_STATE_WAIT;
  }
  else if (flags & AT_LINE_PLUS) {
    /* Comand response with data */
    if (flags & AT_LINE_COLON) {
      /* Line contains colon, string compare can be performed */
      pCb->resp_code = GetCommandCode (pMem);

      if (pCb->resp_code == CMD_IPD) {
        /* Receive network data (+IPD) */
        /* Find colon, there is no CRLF after +IPD */
        pCb->resp_len = (uint8_t)BufFindByte (':', pMem);

        rval = AT_STATE_RESP_DATA;
      }
      else {
        /* Check if line is terminated */
        n = BufFind (crlf, 2, pMem);

        if (n == -1) {
          /* Not terminated, wait for more data */
          rval = AT_STATE_WAIT;
        }
        else {
          /* Line terminator found */
          pCb->resp_len = (uint8_t)n;

          rval = AT_STATE_RESP_DATA;
        }
      }
    }
    else if (flags & AT_LINE_NUMBER) {
      /* Response contains plus and a number, ping response (+x) */
      pCb->resp_code = CMD_PING;

      /* Check if line is terminated */
      n = BufFind (crlf, 2, pMem);

      if (n == -1) {
        /* Not terminated, wait for more data */
        rval = AT_STATE_WAIT;
      }
      else {
        /* Line terminator found */
        pCb->resp_len = (uint8_t)n;

        rval = AT_STATE_RESP_DATA;
      }
    }
    else {
      /* No colon, out of sync */
      rval = AT_STATE_FLUSH;
    }
  }
  else if (flags & AT_LINE_ASCII) {
    /* Line contains ascii characters */
    if (flags & AT_LINE_CRLF) {
      /* Line is terminated, string compare can be performed */
      code = GetGMRResponseCode(pMem);

      if (code != AT_GMR_UNKNOWN) {
        rval = AT_STATE_RESP_GMR;
      }
      else {
        code = GetASCIIResponseCode (pMem);

        if (code == AT_RESP_ECHO) {
          /* Command echo received */
          rval = AT_STATE_RESP_ECHO;
        }
        else if (code != AT_RESP_UNKNOWN) {
          /* Generic response received */
          rval = AT_STATE_RESP_GEN;
        }
        else {
          /* Unknown */
          rval = AT_STATE_FLUSH;
        }
      }

      /* Save response code */
      pCb->msg_code = code;
    }
    else {
      /* No line termination */
      rval = AT_STATE_FLUSH;
    }
  }
  else if (flags & AT_LINE_CTRL) {
    /* Line contains ascii number */
    if (flags & AT_LINE_CRLF) {
      /* Line is terminated, check content */
      code = GetCtrlResponseCode (pMem);

      pCb->ctrl_code = code;

      rval = AT_STATE_RESP_CTRL;
    }
    else {
      /* Out of sync */
      rval = AT_STATE_FLUSH;
    }
  }
  else if (flags & AT_LINE_TXREQ) {
    /* Line contains data request character */
    rval = AT_STATE_SEND_DATA;
  }
  else {
    /* Unknown */
    rval = AT_STATE_FLUSH;
  }

  return (rval);
}


/**
  Compare received data with predefined strings and return corresponding command code.

  \return CommandCode_t
*/
static uint8_t GetCommandCode (BUF_LIST *mem) {
  uint8_t i, maxi, code;
  int32_t  val;

  code = CMD_UNKNOWN;
  maxi = sizeof(List_PlusResp)/sizeof(List_PlusResp[0]);

  for (i = 0; i < maxi; i++) {
    val = BufCompareString (List_PlusResp[i].str, 1U, mem);

    if (val > 0) {
      /* String matches */
      code = i;
      break;
    }
  }
  return (code);
}


/**
  Compare received data with predefined strings and return corresponding response code.

  \return Generic response code, see AT_RESP_ definitions
*/
static uint8_t GetASCIIResponseCode (BUF_LIST *mem) {
  uint8_t i, maxi, code;
  int32_t val;

  code = AT_RESP_UNKNOWN;
  maxi = sizeof(List_ASCIIResp)/sizeof(List_ASCIIResp[0]);

  for (i = 0; i < maxi; i++) {
    /* Search for responses (OK, ERROR, FAIL, SEND OK, ...) */
    val = BufCompareString (List_ASCIIResp[i].str, 0U, mem);

    if (val > 0) {
      /* String matches */
      code = i;
      break;
    }
  }

  return (code);
}

/**
  Compare received data with predefined strings and return corresponding response code.

  \return GMR code, see ESP_GMR_ definitions
*/
static uint8_t GetGMRResponseCode (BUF_LIST *mem) {
  uint8_t i, maxi, code;
  int32_t val;

  code = AT_GMR_UNKNOWN;
  maxi = sizeof(List_Gmr)/sizeof(List_Gmr[0]);

  for (i = 0; i < maxi; i++) {
    /* Search for responses */
    val = BufCompareString (List_Gmr[i].str, 0U, mem);

    if (val > 0) {
      /* String matches */
      code = i;
      break;
    }
  }

  return (code);
}


/**
  Compare received data with predefined strings and return corresponding control code.

  Currently supported control strings:
  <conn id>,CONNECT
  <conn id>,CLOSED

  \return ESP_CTRL_CONNECT, ESP_CTRL_CLOSED
*/
//static uint32_t GetCtrlResponseCode (BUF_LIST *mem) {
static uint8_t GetCtrlResponseCode (BUF_LIST *mem) {
  uint8_t i, maxi, code;
  int32_t val;

  code = AT_CTRL_UNKNOWN;
  maxi = sizeof(List_Ctrl)/sizeof(List_Ctrl[0]);

  for (i = 0; i < maxi; i++) {
    val = BufCompareString (List_Ctrl[i].str, 2U, mem);

    if (val > 0) {
      /* String matches */
      code = i;
      break;
    }
  }
  return (code);
}

/* ------------------------------------------------------------------------- */

/**
  Get response argument.

  The return value should indicate continuation pattern. For example when there
  are multiple responses, as
  +CWLAP:<ecn>,<ssid>,<rssi>,<mac>,<ch>,<freq offset>\r\n
  +CWLAP:<ecn>,<ssid>,<rssi>,<mac>,<ch>,<freq offset>\r\n
  +CWLAP:<ecn>,<ssid>,<rssi>,<mac>,<ch>,<freq offset>\r\n
  +CWLAP:<ecn>,<ssid>,<rssi>,<mac>,<ch>,<freq offset>\r\nOK

  the return value should indicate what follows after \r\n termination:
  - in case if '+' follows, there is another response to be processed
  - in case if "OK" follows, response was processed completely.

  Note that +IPD response format is different and there is no \r\n terminator.

  \return -2: failed, specified buffer too small (sz of buf)
          -1: response incomplete, rx buffer empty
           0: retrieved, last delimiter: ','
           1: retrieved, last delimiter: ':'
           2: retrieved, last delimiter: '\r', response pending ('+')
           3: retrieved, last delimiter: '\r', last response ("OK")
*/
static int32_t GetRespArg (uint8_t *buf, uint32_t sz) {
  uint32_t i;   /* argument size    */
  uint32_t str; /* string indicator */
  int32_t val;
  uint8_t b;
  uint8_t d[] = {',', ':', '\r'};
  int32_t del;

  if (BufPeekByte(&(pCb->resp)) == '+') {
    /* Sync till the first ':' after +command string */
    do {
      val = BufReadByte (&(pCb->resp));
      
      if (val == -1) {
        return -1;
      }
      if (val == ',') {
        /* Handle "+IPD," response format */
        break;
      }
    }
    while (val != ':');
  }

  /* Initialize number of delimiters, string indicator (str) and argument size (i) */
  del = sizeof(d);
  str = 0U;
  i   = 0U;

  do {
    if (i == sz) {
      /* Specified buffer too small */
      val = -2;
    }
    else {
      /* Read one byte from response buffer */
      val = BufReadByte (&(pCb->resp));
    }

    if (val < 0) {
      break;
    }

    b = (uint8_t)val;

    if (b == '"') {
      /* Toggle string indicator */
      str ^= 1U;
    }

    if ((str == 0U) && ((b == '(') || (b == ')'))) {
      /* Ignore characters if not within string */
    }
    else {
      if (str == 0U) {
        /* Check delimiters (when outside of string) */
        for (val = 0; val < del; val++) {
          if (b == d[val]) {
            /* Found delimiter, set null terminator */
            b = '\0';
            break;
          }
        }
      }
      buf[i] = b;

      i++;
    }
  }
  while (val >= del);

  if (val == 2) {
    /* Check if this is the last response */

    /* Clear '\n' character */
    BufFlushByte (&(pCb->resp));

    /* Peek what is next */
    b = (uint8_t)BufPeekByte (&(pCb->resp));

    if (b != '+') {
      val = 3;
    }
  }

  return (val);
}

/* ------------------------------------------------------------------------- */


/**
  Retrieve incomming data.

  Response: +IPD,<link ID>,<len>[,<remote IP>, <remote port>]:<data>

  This response does not have CRLF terminator, <len> is the number of bytes in <data>.
  Also note that the format of +IPD is also different in how argument are provided.

  \param[out]  link_id     connection ID
  \param[out]  len         data length
  \param[out]  remote_ip   remote IP (enabled by command AT+CIPDINFO=1)
  \param[out]  remote_port remote port (enabled by command AT+CIPDINFO=1)
  

  \return 0: ok, len of data shall be read from buffer
          negative: buffer empty or packet incomplete
*/
int32_t AT_Resp_IPD (uint32_t *link_id, uint32_t *len, uint8_t *remote_ip, uint16_t *remote_port) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    p = (char *)&buf[0];

    /* Got valid argument */
    if (a == 0) {
      /* Read <link ID> (buf = integer) */
      uval = strtoul (p, &p, 10);

      *link_id = uval;
    }
    else if (a == 1) {
      /* Read <len> (buf = integer) */
      uval = strtoul (p, &p, 10);

      *len = uval;
    }
    else if (a == 2) {
      /* Read <remote_ip> (buf = "xxx.xxx.xxx.xxx") */
      if (remote_ip != NULL) {
        AT_Parse_IP (p, remote_ip);
      }
    }
    else if (a == 3) {
      /* Read <remote_port> (buf = integer?) */
      uval = strtoul (p, &p, 10);

      if (remote_port != NULL) {
        *remote_port = (uint16_t)uval;
      }
    }
    else {
      /* ??? */
      break;
    }

    /* Increment number of arguments */
    a++;

    if (val == 1) {
      /* At the ':' delimiter */
      val = 0;
      break;
    }
  }
  while (val >= 0);

  return (val);
}


/**
  Get +LINK_CONN response parameters (see +SYSMSG_CUR).

  +LINK_CONN:<status_type>,<link_id>,"UDP/TCP/SSL",<c/s>,<remote_ip>,
                          <remote_port>,<local_port>
*/
int32_t AT_Resp_LinkConn (uint32_t *status, AT_DATA_LINK_CONN *conn) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
        /* Read <status_type> (buf = integer) */
        uval = strtoul (p, &p, 10);

        *status = uval;
      }
      else if (a == 1) {
        /* Read <link_id> (buf = integer) */
        uval = strtoul (p, &p, 10);

        conn->link_id = (uint8_t)uval;
      }
      else if (a == 2) {
        /* Read type string "UDP/TCP/SSL" */
        strcpy (conn->type, p);
      }
      else if (a == 3) {
        /* Read client/server flag */
        uval = strtoul (p, &p, 10);

        conn->c_s = (uint8_t)uval;
      }
      else if (a == 4) {
        /* Read <remote_ip> (buf = "xxx.xxx.xxx.xxx") */
        AT_Parse_IP (p, conn->remote_ip);
      }
      else if (a == 5) {
        /* Read <remote_port> (buf = integer) */
        uval = strtoul (p, &p, 10);

        conn->remote_port = (uint16_t)uval;
      }
      else if (a == 6) {
        /* Read <local_port> (buf = integer) */
        uval = strtoul (p, &p, 10);

        conn->local_port = (uint16_t)uval;
      }
      else {
        /* ??? */
        break;
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while ((val != 2) && (val != 3));

  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}

/**
  Get connection number from the <conn_id>,CONNECT or <conn_id>,CLOSED response.
  
  \param[in]  conn_id   connection ID
  \return execution status:
          -1: no response (buffer empty)
           0: connection number retrieved
*/
int32_t AT_Resp_CtrlConn (uint32_t *conn_id) {
  int32_t val;
  uint8_t b;

  val = BufReadByte (pMem);

  if (val != -1) {
    b = (uint8_t)val;

    *conn_id = b - '0';
    val = 0;
  }

  return (val);
}

/**
  Get +STA_CONNECTED and +STA_DISCONNECTED response (mac).

  +STA_CONNECTED:<sta_mac>crlf
  +STA_DISCONNECTED"<sta_mac>crlf
*/
int32_t AT_Resp_StaMac (uint8_t mac[]) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;

  /* Retrieve response argument */
  val = GetRespArg (buf, sizeof(buf));

  if (val > 1) {
    p = (char *)&buf[0];

    /* Read <sta_mac> (buf = "xx:xx:xx:xx:xx:xx") */
    AT_Parse_MAC (p, mac);

    val = 0;
  }

  return (val);
}

/**
  Get ERR_CODE:0x... response.

  \param[out] err_code    Pointer to 32-bit variable where error code will be stored.
  \return execution status:
          -1: no response (buffer empty)
           0: error code retrieved
*/
int32_t AT_Resp_ErrCode (uint32_t *err_code) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t uval;  /* Unsigned value storage */

  /* Retrieve response argument */
  val = GetRespArg (buf, sizeof(buf));

  if (val > 1) {
    p = (char *)&buf[0];

    /* Read error code (buf = hex integer) */
    uval = strtoul (p, &p, 16);

    *err_code = uval;

    val = 0;
  }

  return (val);
}


/**
  Get standalone generic response.

  Standalone generic responses are responses that come without +CMD:data. Parser detect them and
  deliver them into internal variable.

  \return generic response code AT_RESP_x
*/
int32_t AT_Resp_Generic (void) {

  /* Return generic response */
  return (pCb->gen_resp);
}


/**
  Test AT startup

  Generic response is expected.

  \return 0:OK, -1: error
*/
int32_t AT_Cmd_TestAT (void) {
  char out[8];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "%s", "AT");

  /* Append CRLF and send command */
  return (CmdSend(CMD_UART_CUR, out, n));
}

/**
  Restarts the module

  Generic response is expected.

  Format: AT+RST

  \return 0:OK, -1: error
*/
int32_t AT_Cmd_Reset (void) {
  char out[16];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_RST, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_RST, out, n));
}

/**
  Check version information

  Generic response is expected.

  Format: AT+GMR

  \return 0:OK, -1: error
*/
int32_t AT_Cmd_GetVersion (void) {
  char out[16];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_GMR, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_GMR, out, n));
}

/**
  Get response to GetVersion command

  \param[out] buf   data buffer 
  \param[in]  len   data buffer size
*/
int32_t AT_Resp_GetVersion (uint8_t *buf, uint32_t len) {
  uint8_t crlf[] = {'\r', '\n'};
  int32_t cnt = (int32_t)len;
  int32_t val;
  int32_t n;

  /* Initialize total number of read bytes */
  val = 0U;

  while (val < cnt) {
    /* Check if we can find crlf */
    n = BufFind (crlf, 2, &(pCb->resp));

    if (n < 0) {
      break;
    }

    /* Add crlf */
    n += 2;

    /* Check if len is ok */
    if ((val + n) > cnt) {
      n = (cnt - val);
    }
    val += BufRead(&buf[val], (uint32_t)n, &(pCb->resp));
  }

  /* Flush any leftovers */
  do {
    n = BufReadByte(&(pCb->resp));
  }
  while (n != -1);
 
  return (val);
}


/**
  Enable or disable command echo.

  Received commands can be echoed.
  Generic response is expected.
  
  \param[in]  enable  Echo enable(1) or disable(0)
  \return 0:OK, -1: error
*/
int32_t AT_Cmd_Echo (uint32_t enable) {
  char out[8];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "%s%d", "ATE", enable);

  /* Append CRLF and send command */
  return (CmdSend(CMD_UART_CUR, out, n));
}

/**
  Set/Query the current UART configuration
  
  Format S: AT+UART_CUR=<baudrate>,<databits>,<stopbits>,<parity>,<flow control>
  Format Q: AT+UART_CUR?

  Example S: AT+UART_CUR=115200,8,1,0,0\r\n
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  baudrate
  \param[in]  databits
  \param[in]  stop_par_flowc stopbits[5:4], parity[3:2], flow control[1:0]
  \return 0:OK, -1: error
*/
int32_t AT_Cmd_ConfigUART (uint32_t at_cmode, uint32_t baudrate, uint32_t databits, uint32_t stop_par_flowc) {
  char out[32];
  uint32_t stopbits, parity, flow_ctrl;
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_UART_CUR, AT_CMODE_SET, out);

  if (at_cmode == AT_CMODE_SET) {
    stopbits  = (stop_par_flowc >> 4) & 0x3;
    parity    = (stop_par_flowc >> 2) & 0x3;
    flow_ctrl = (stop_par_flowc >> 0) & 0x3;

    /* Add command arguments */
    n += sprintf (&out[n], "%d,%d,%d,%d,%d", baudrate, databits, stopbits, parity, flow_ctrl);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_UART_CUR, out, n));
}

/**
  Get response to ConfigUART command

  Response Q: +UART_CUR:<baudrate>,<databits>,<stopbits>,<parity>,<flow control>\r\n\r\n\OK

  \param[out] baudrate
  \param[out] databits
  \param[out] stop_par_flowc stopbits[5:4], parity[3:2], flow control[1:0]
  \return
*/
int32_t AT_Resp_ConfigUART (uint32_t *baudrate, uint32_t *databits, uint32_t *stop_par_flowc) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */
  uint32_t spf;

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
        /* Read <baudrate> */
        uval = strtoul (p, &p, 10);

        /* Note: if S was 115200, Q might return 115273 */
        *baudrate = uval;
      }
      else if (a == 1) {
        /* Read <databits> */
        uval = strtoul (p, &p, 10);

        *databits = uval;
      }
      else if (a == 2) {
        /* Read <stopbits> */
        uval = strtoul (p, &p, 10);

        spf = (uval & 0x3) << 4;
      }
      else if (a == 3) {
        /* Read <parity> */
        uval = strtoul (p, &p, 10);

        spf = (uval & 0x3) << 2;
      }
      else if (a == 4) {
        /* Read <flow control> */
        uval = strtoul (p, &p, 10);

        spf = (uval & 0x3);

        *stop_par_flowc = spf;
      }
      else {
        /* Ignore unknown arguments */
        break;
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while ((val != 2) && (val != 3));

  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}

/**
  Configure the sleep modes.

  Format: AT+SLEEP=<sleep mode>

  \note Command can be used only in Station mode. Modem-sleep is the default mode.

  \param[in]  sleep_mode  sleep mode (0: disabled, 1: Light-sleep, 2: Modem-sleep)
  \return 0: ok, -1: error
*/
int32_t AT_Cmd_Sleep (uint32_t at_cmode, uint32_t sleep_mode) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_SLEEP, AT_CMODE_SET, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", sleep_mode);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_SLEEP, out, n));
}

/**
  Get response to AutoConnectAP command.

  Response Q: +SLEEP:<sleep mode>
  Example  Q: +SLEEP:2\r\n\r\nOK\r\n\

  \param[out]   sleep_mode  Pointer to variable the where sleep mode is stored
  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_Sleep (uint32_t *sleep_mode) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      /* Read <sleep mode> */
      *sleep_mode = p[0] - '0';
      break;
    }
  }
  while (val != 3);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}

/**
  Set maximum value of RF TX power (dBm).

  Power range for
  ESP8266: range [0:82], units 0.25dBm
  ESP32:
    AT 1.2.0: range[0:11] = {19.5, 19, 18.5, 17, 15, 13, 11, 8.5, 7, 5, 2, -1}dBm
    AT 2.0.0: range[40,82], units 0.25dBm (value 78 means RF power 78*0.25dBm = 19.5dBm)

  Response: Generic

  \param[in]  tx_power  power value
  \return 0: ok, -1: error
*/
int32_t AT_Cmd_TxPower (uint32_t tx_power) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_RFPOWER, AT_CMODE_SET, out);

  /* Add command arguments */
  n += sprintf (&out[n], "%d", tx_power);

  /* Append CRLF and send command */
  return (CmdSend(CMD_RFPOWER, out, n));
}

/**
  Set current system messages.

  Command: SYSMSG/SYSMSG_CUR
  Response: Generic

  Bit 0: configure the message of quitting passthrough transmission
  Bit 1: configure the message of establishing a network connection
         0 - <Link ID>,CONNECT
         1 - +LINK_CONN:<status_type>,<link_id>,"UDP/TCP/SSL",<remote_ip>,
                        <remote_port>,<local_port>

  \note Only AT set command is available.

  \param[in]  n         message configuration bit mask [0:1]
*/
int32_t AT_Cmd_SysMessages (uint32_t n) {
  char out[32];
  int32_t k;

  /* Open AT command (AT+<cmd><mode> */
  k = CmdOpen (CMD_SYSMSG_CUR, AT_CMODE_SET, out);

  /* Add command arguments */
  k += sprintf (&out[k], "%d", n);

  /* Append CRLF and send command */
  return (CmdSend(CMD_SYSMSG_CUR, out, k));
}


/**
  Set/Query the current Wi-Fi mode

  Format S: AT+CWMODE_CUR=<mode>

  Response Q: AT_Resp_CurrentMode
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      Mode, 0: RF disabled (ESP32), 1: station, 2: soft AP, 3: soft AP + station
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_CurrentMode (uint32_t at_cmode, uint32_t mode) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWMODE_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", mode);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWMODE_CUR, out, n));
}

/**
  Get response to CurrentMode command

  Response Q: +CWMODE_CUR:<mode>
  Example  Q: +CWMODE_CUR:3\r\n\r\n\OK

  \param[in]  mode      Mode, 1: station, 2: soft AP, 3: soft AP + station
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Resp_CurrentMode (uint32_t *mode) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      /* Read <mode> */
      *mode = p[0] - '0';
      break;
    }
  }
  while (val != 2);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Set/Query Configures the Name of Station

  Format S: AT+CWHOSTNAME=<hostname>
  Format Q: AT+CWHOSTNAME?

  Response S:
  "OK"
  "ERROR"

  Response Q: Current HostName
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  hostname  the host name of the Station, the maximum length is 32 bytes.
  \return 0: OK, -1: ERROR
*/
int32_t AT_Cmd_HostName (uint32_t at_cmode, const char* hostname) {
  char out[15 + 32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWHOSTNAME, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%s\"", hostname);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWHOSTNAME, out, n));
}

/**
  Get response to HostName command

  Response Q: +CWHOSTNAME:<host	name>
  Example  Q: +CWHOSTNAME:ESP_XXXXXX\r\n\r\nOK

  \param[in]  hostname  the host name of the Station, the maximum length is 32 bytes.
  \return 0: OK, -1: ERROR
*/
int32_t AT_Resp_HostName (char* hostname) {
  uint8_t buf[12 + 32 + 6];
  int32_t val;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      strcpy (hostname, (const char *)buf);
      break;
    }
  }
  while (val != 2);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Set/Query connected access point or access point to connect to

  Format S: AT+CWJAP_CUR=<ssid>,<pwd>[,<bssid>]
  Format Q: AT+CWJAP_CUR?

  Response S:
  "WIFI CONNECTED"
  "WIFI GOT IP"
  ""
  "OK"

  Response Q: AT_Resp_ConnectAP

  \param[in]  ssid
  \param[in]  pwd
  \param[in]  bssid
  \return 0: ok, -1: error
*/
int32_t AT_Cmd_ConnectAP (uint32_t at_cmode, const char *ssid, const char *pwd, const uint8_t *bssid) {
  char out[113+1];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWJAP_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%s\",\"%s\"", ssid, pwd);

    if (bssid != NULL) {
      n += sprintf (&out[n], ",\"%02x:%02x:%02x:%02x:%02x:%02x\"", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWJAP_CUR, out, n));
}

/**
  Response to ESP_ConnectAP command.

  Response Q: +CWJAP_CUR:<ssid>,<bssid>,<channel>,<rssi>
  Example  Q: +CWJAP_CUR:"AP_SSID","xx:xx:xx:xx:xx:xx",6,-60

  Set response is handled by using NULL as ap argument.

  \return (SET response)
           - 1: connection timeout
           - 2: wrong password
           - 3: cannot find the target AP
           - 4: connection failed
          (QUERY response)
           execution status
           - negative: error
           - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_ConnectAP (AT_DATA_CWJAP *ap) {
  char    *p;
  uint8_t  buf[32+1];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (ap == NULL) {
      /* Extract and return error code */
      return (buf[0] - 0x30);
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
      /* Read <ssid> (buf = "string") */
        strcpy (ap->ssid, (const char *)buf);
      }
      else if (a == 1) {
        /* Read <bssid> (buf = "xx:xx:xx:xx:xx:xx") */
        AT_Parse_MAC (p, ap->bssid);
      }
      else if (a == 2) {
        /* Read <ch> */
        uval = strtoul (p, &p, 10);

        ap->ch = (uint8_t)uval;
      }
      else if (a == 3) {
        /* Read <rssi> */
        uval = strtoul (p, &p, 10);
        
        ap->rssi = (uint8_t)uval;
      }
      else {
        /* Unknown arguments */
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while ((val != 2) && (val != 3));

  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}


/**
  Disconnect from current Access Point (CWQAP)

  \return 0:ok, -1: error
*/
int32_t AT_Cmd_DisconnectAP (void) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWQAP, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWQAP, out, n));
}


/**
  Configure local access point (SoftAP must be active)
  
  Format: AT+CWSAP_CUR=<ssid>,<pwd>,<chl>,<ecn>[,<max conn>][,<ssid hidden>]
*/
int32_t AT_Cmd_ConfigureAP (uint32_t at_cmode, AT_DATA_CWSAP *cfg) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWSAP_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%s\",\"%s\",%d,%d", cfg->ssid, cfg->pwd, cfg->ch, cfg->ecn);

    /* Add optional arguments */
    n += sprintf (&out[n], ",%d,%d", cfg->max_conn, cfg->ssid_hide);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWSAP_CUR, out, n));
}

/**
  Get response to ConfigureAP command

  Format: AT+CWSAP_CUR=<ssid>,<pwd>,<chl>,<ecn>,<max conn>,<ssid hidden>

  \param[in]  cfg       AP configuration structure
  \return
*/
int32_t AT_Resp_ConfigureAP (AT_DATA_CWSAP *cfg) {
  char    *p;
  uint8_t  buf[32+1];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
      /* Read <ssid> */
        strcpy (cfg->ssid, (const char *)buf);
      }
      else if (a == 1) {
        /* Read <pwd> */
        strcpy (cfg->pwd, (const char *)buf);
      }
      else if (a == 2) {
        /* Read <ch> */
        uval = strtoul (p, &p, 10);

        cfg->ch = (uint8_t)uval;
      }
      else if (a == 3) {
        /* Read <ecn> */
        cfg->ecn = p[0] - '0';
      }
      else if (a == 4) {
        /* Read <max conn> */
        cfg->max_conn = p[0] - '0';

      }
      else if (a == 5) {
        /* Read <ssid hidden> */
        cfg->ssid_hide = p[0] - '0';
      }
      else {
        /* ??? */
        break;
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while ((val != 2) && (val != 3));

  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}


/**
  List available Access Points (CWLAP)
*/
int32_t AT_Cmd_ListAP (void) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWLAP, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWLAP, out, n));
}

/**
  Process input string

  Format: +CWLAP:<ecn>,<ssid>,<rssi>,<mac>,<ch>,<freq offset>\r\nOK (AT 0.30)
  Format: +CWLAP:<ecn>,<ssid>,<rssi>,<mac>,<ch>,<freq offset>,
                 <freq cali>,<pairwise_cipher>,<group_cipher>,<bgn>,<wps>\r\n\OK (AT 2.2.0)

  \return execution status
          - negative: error
          - 0: access point list is empty
          - 1: access point list contains more data
*/
int32_t AT_Resp_ListAP (AT_DATA_CWLAP *ap) {
  char    *p;
  uint8_t  buf[32+1];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != 1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
        /* Read <ecn> */
        ap->ecn = p[0] - '0';
      }
      else if (a == 1) {
        /* Read <ssid> */
        strcpy (ap->ssid, (const char *)buf);
      }
      else if (a == 2) {
        /* Read <rssi> */
        uval = strtoul (p, &p, 10);

        ap->rssi = (int8_t)uval;
      }
      else if (a == 3) {
        /* Read <mac> (p == "xx:xx:xx:xx:xx:xx") */
        AT_Parse_MAC (p, ap->mac);
      }
      else if (a == 4) {
        /* Read <ch> */
        uval = strtoul (p, &p, 10);

        ap->ch = (uint8_t)uval;
      }
      else if (a == 5) {
        /* Get <freq offset> */
        uval = strtoul (p, &p, 10);

        ap->freq_offs = (uint16_t)uval;
      }
      else {
        /* Ignore unknown arguments */
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while ((val != 2) && (val != 3));
  
  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}


/**
  Set station MAC address

  Do not set the same MAC address for Station and SoftAP.

  Format: AT+CIPSTAMAC_CUR=<mac>

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mac       Pointer to 6 byte array containing MAC address
*/
int32_t AT_Cmd_StationMAC (uint32_t at_cmode, const uint8_t mac[]) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSTAMAC_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%02X:%02X:%02X:%02X:%02X:%02X\"",
                            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSTAMAC_CUR, out, n));
}

/**
  Get response to StationMAC command

  String form: +CIPSTAMAC_CUR:<mac>\r\n\r\nOK

  \param[out]   mac   Pointer to 6 byte array where MAC address will be stored
  \return execution status
          -1 : error
           0 : MAC retrieved
*/
int32_t AT_Resp_StationMAC (uint8_t mac[]) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to MAC string: "xx:xx:xx:xx:xx:xx" */
      p = (char *)&buf[0];

      /* Parse MAC string */
      AT_Parse_MAC (p, mac);
    }
  }
  while (val != 3);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Set/Query the Access Point MAC address

  Do not set the same MAC address for Station and SoftAP.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mac       Pointer to 6 byte array containing MAC address
  \return 
*/
int32_t AT_Cmd_AccessPointMAC (uint32_t at_cmode, uint8_t mac[]) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPAPMAC_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%02X:%02X:%02X:%02X:%02X:%02X\"",
                            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPAPMAC_CUR, out, n));
}

/**
  Get response to AccessPointMAC command
  
  \param[out]   mac   Pointer to 6 byte array where MAC address will be stored
  \return execution status
*/
int32_t AT_Resp_AccessPointMAC (uint8_t mac[]) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to MAC string: "xx:xx:xx:xx:xx:xx" */
      p = (char *)&buf[0];

      /* Parse MAC string, skip initial quote */
      AT_Parse_MAC (p, mac);
    }
  }
  while (val != 3);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Set/Query current IP address of the local station

  Format S: AT+CIPSTA_CUR=<ip>,<gateway>,<netmask>
  Format Q: AT+CIPSTA_CUR?

  Example: AT+CIPSTA_CUR="192.168.1.100","192.168.6.1","255.255.255.0"

  Response: AT_Resp_StationIP
*/
int32_t AT_Cmd_StationIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]) {
  char out[70];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSTA_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%d.%d.%d.%d\"", ip[0], ip[1], ip[2], ip[3]);
    
    if (gw != NULL) {
      /* Add gateway */
      n += sprintf (&out[n], ",\"%d.%d.%d.%d\"", gw[0], gw[1], gw[2], gw[3]);

      if (mask != NULL) {
        /* Add netmask */
        n += sprintf (&out[n], ",\"%d.%d.%d.%d\"", mask[0], mask[1], mask[2], mask[3]);
      }
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSTA_CUR, out, n));
}

/**
  Get response to StationIP command

  Response: +CIPSTA_CUR:<ip>
            +CIPSTA_CUR:<gateway>
            +CIPSTA_CUR:<netmask>

  Example:  +CIPSTA_CUR:ip:"192.168.1.155"

  \param[out]   addr    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
int32_t AT_Resp_StationIP (uint8_t addr[]) {
  uint8_t  buf[32]; /* Argument buffer */
  int32_t  val;     /* Control value */
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != 1) {
      /* Set pointer to start of string */
      p = (char *)&buf[0];

      /* Parse IP address (xxx.xxx.xxx.xxx) */
      AT_Parse_IP (p, addr);
    }
  }
  while ((val != 2) && (val != 3));

  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}

/**
  Set/Query current IP address of the local access point

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  ip    IP address
  \param[in]  gw    Gateway address
  \param[in]  mask  Netmask
  \return 
*/
int32_t AT_Cmd_AccessPointIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]) {
  char out[70];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPAP_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%d.%d.%d.%d\"", ip[0], ip[1], ip[2], ip[3]);
    
    if (gw != NULL) {
      /* Add gateway */
      n += sprintf (&out[n], ",\"%d.%d.%d.%d\"", gw[0], gw[1], gw[2], gw[3]);

      if (mask != NULL) {
        /* Add netmask */
        n += sprintf (&out[n], ",\"%d.%d.%d.%d\"", mask[0], mask[1], mask[2], mask[3]);
      }
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPAP_CUR, out, n));
}

int32_t AT_Resp_AccessPointIP (uint8_t addr[]) {
  return AT_Resp_StationIP (addr);
}


/**
  Set user defined DNS servers

  Format S: AT+CIPDNS_CUR=<enable>[,<DNS server0>,<DNS server1>]
  Format S: AT+CIPDNS_CUR?

  Example: AT+CIPDNS_CUR=1,"192.168.0.1"
  
  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable    Enable (1) or disable (0) user defined DNS servers
  \param[in]  dns0      Primary DNS server
  \param[in]  dns1      Secondary DNS server
*/
int32_t AT_Cmd_DNS (uint32_t at_cmode, uint32_t enable, uint8_t dns0[], uint8_t dns1[]) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPDNS_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", enable);
    
    if (dns0 != NULL) {
      /* Add DNS 0 */
      n += sprintf (&out[n], ",\"%d.%d.%d.%d\"", dns0[0], dns0[1], dns0[2], dns0[3]);

      if (dns1 != NULL) {
        /* Add DNS 1 */
        n += sprintf (&out[n], ",\"%d.%d.%d.%d\"", dns1[0], dns1[1], dns1[2], dns1[3]);
      }
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPDNS_CUR, out, n));
}

#if (AT_VARIANT == AT_VARIANT_ESP32) && (AT_VERSION >= AT_VERSION_2_0_0_0)
/**
  Get response to DNS command

  Example: +CIPDNS=1,"192.168.0.1","192.168.0.2"\r\n

  \param[out]   enable  Pointer to variable where enable flag will be stored
  \param[out]   dns0    Pointer to 4 byte array where DNS 0 address will be stored
  \param[out]   dns1    Pointer to 4 byte array where DNS 1 address will be stored
  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_DNS (uint32_t *enable, uint8_t dns0[], uint8_t dns1[]) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
      /* Read <lease time> (buf = integer) */
        uval = strtoul (&p[0], &p, 10);

        if (enable != NULL) {
          *enable = uval;
        }
      }
      else if (a == 1) {
        /* Parse DNS0 */
        AT_Parse_IP (p, dns0);
      }
      else if (a == 2) {
        /* Parse DNS1 */
        AT_Parse_IP (p, dns1);
      }
      else {
        /* ??? */
        break;
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while (val != 3);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}
#else
/**
  Get response to DNS command

  Example: +CIPDNS_CUR=192.168.0.1

  \param[out]   addr    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
int32_t AT_Resp_DNS (uint8_t addr[]) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to start of string */
      p = (char *)&buf[0];
      
      /* Parse IP address */
      AT_Parse_IP (p, addr);
      break;
    }
  }
  while ((val != 2) && (val != 3));

  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}
#endif


#if (AT_VARIANT == AT_VARIANT_ESP32)
/**
  Set/Query DHCP state

  Query: AT+CWDHCP?
  Set:   AT+CWDHCP=<operate>,<mode>

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  operate   0: disable DHCP, 1: enable DHCP
  \param[in]  mode      Bit0: set station, Bit1: set soft-ap
*/
int32_t AT_Cmd_DHCP (uint32_t at_cmode, uint32_t operate, uint32_t mode) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWDHCP_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d,%d", operate, mode);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWDHCP_CUR, out, n));
}
#else
/**
  Set/Query DHCP state

  Format S: AT+CWDHCP_CUR=<mode>,<en>
  Format Q: AT+CWDHCP_CUR?

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0: set soft-ap, 1: set station, 2: set both (soft-ap and station)
  \param[in]  enable    0: disable DHCP, 1: enable DHCP
*/
int32_t AT_Cmd_DHCP (uint32_t at_cmode, uint32_t mode, uint32_t enable) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWDHCP_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d,%d", mode, enable);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWDHCP_CUR, out, n));
}
#endif


/**
  Get response to DHCP command

  Response Q: +CWDHCP_CUR:<mode>
  Example  Q: +CWDHCP_CUR:3\r\n\r\n\OK

  \param[out]   mode    Pointer to variable the DHCP mode is stored
  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_DHCP (uint32_t *mode) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      /* Read <mode> */
      *mode = (p[0] - '0') & 0x03U;
      break;
    }
  }
  while (val != 3);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Set/Query DHCP IP address lease time and range for local access point

  AT command is enabled when device runs as SoftAP, and when DHCP is enabled.
  The IP address should be in the same network segment as the IP address of device 
  SoftAP. SoftAP IP address must be different than <start IP> or <end IP>.

  Format S: AT+CWDHCPS_CUR=<enable>,<lease time>,<start IP>,<end IP>
  Format Q: AT+CWDHCPS_CUR?

  Example: AT+CWDHCPS_CUR=1,3,"192.168.4.10","192.168.4.15"

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  en_tlease   bit[16] 0: disable setting, 1: enable setting the IP range
                          bits[15:0] lease time in minutes, range [1, 2880]
  \param[in]  ip_start    first IP in range that can be obtained from DHCP server
  \param[in]  ip_end      last IP in range that can be obtained from DHCP server
*/
int32_t AT_Cmd_RangeDHCP (uint32_t at_cmode, uint32_t en_tlease, uint8_t ip_start[], uint8_t ip_end[]) {
  char out[64];
  int32_t n;
  uint32_t en      = en_tlease >> 16;
  uint32_t t_lease = en_tlease & 0xFFFF;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWDHCPS_CUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    if (en != 0) {
      n += sprintf (&out[n], "%d,%d,\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"",
                             en,
                             t_lease,
                             ip_start[0], ip_start[1], ip_start[2], ip_start[3],
                             ip_end[0],   ip_end[1],   ip_end[2],   ip_end[3]);
    }
    else {
     /* AT+CWDHCPS_CUR=0 */
     n += sprintf (&out[n], "%d", en);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWDHCPS_CUR, out, n));
}

/**
  Get response to RangeDHCP command

  Response Q: +CWDHCPS_CUR=<lease time>,<start IP>,<end IP>
  Example  Q: +CWDHCPS_CUR:???

  \param[out]  t_lease  lease time in minutes, range[1, 2880]
  \param[out]  ip_start first IP in range that can be obtained from DHCP server
  \param[out]  ip_end   last IP in range that can be obtained from DHCP server

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_RangeDHCP (uint32_t *t_lease, uint8_t ip_start[], uint8_t ip_end[]) {
  char    *p;
  uint8_t  buf[32+1];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
      /* Read <lease time> (buf = integer) */
        uval = strtoul (&p[0], &p, 10);

        *t_lease = uval;
      }
      else if (a == 1) {
        /* Read <start IP> (buf = "xxx.xxx.xxx.xxx") */
        AT_Parse_IP (p, ip_start);
      }
      else if (a == 2) {
        /* Read <end IP> (buf = "xxx.xxx.xxx.xxx") */
        AT_Parse_IP (p, ip_end);
      }
      else {
        /* ??? */
        break;
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while (val != 3);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Set/Query Auto-Connect to the AP

  Format: AT+CWAUTOCONN=<enable>

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable    0:disable, 1:enable auto-connect on power-up

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_AutoConnectAP (uint32_t at_cmode, uint32_t enable) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWAUTOCONN, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command argument */
    n += sprintf (&out[n], "%d", enable);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWAUTOCONN, out, n));
}

/**
  Get response to AutoConnectAP command

  Response Q: +CWAUTOCONN:<enable>
  Example  Q: +CWAUTOCONN:0\r\n\OK\r\n\

  \param[out]   enable  Pointer to variable the enable status is stored
  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_AutoConnectAP (uint32_t *enable) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      /* Read <mode> */
      *enable = p[0] - '0';
      break;
    }
  }
  while (val != 3);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Retrieve the list of IP (stations) connected to access point

  Format: AT+CWLIF
*/
int32_t AT_Cmd_ListIP (void) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CWLIF, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_CWLIF, out, n));
}

/**
  Get response to ListIP command

  Format: +CWLIF=<IP addr>,<mac>
  
  Example: +CWLIF:192.168.4.2,xx:xx:xx:xx:xx:xx

  \param[out] ip  Pointer to 4 byte array where IP address will be stored
  \param[out] mac Pointer to 6 byte array where MAC address will be stored
  \return execution status
          - negative: error
          - 0: list is empty
          - 1: list contains more data
*/
int32_t AT_Resp_ListIP (uint8_t ip[], uint8_t mac[]) {
  uint8_t  buf[32]; /* Argument buffer */
  int32_t  val;     /* Control value */
  uint32_t a;       /* Argument counter */
  char *p;

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (&buf[1], sizeof(buf)-1);

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != 1) {
      /* Set pointer to start of string */
      p = (char *)&buf[0];

      if (a == 0) {
        /* Parse IP address (xxx.xxx.xxx.xxx) */
        AT_Parse_IP (p, ip);
      }
      else {
        /* Parse MAC string (xx:xx:xx:xx:xx:xx) */
        AT_Parse_MAC (p, mac);
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while (val != 2);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}

/* TCP/IP Related AT Commands ---------------------- */

/**
  Get the connection status.

  Format: AT+CIPSTATUS

  \note Only AT command execute mode is available (AT_CMODE_EXEC)

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_GetStatus (uint32_t at_cmode) {
  char out[64];
  int32_t n;

  if (at_cmode != AT_CMODE_EXEC) {
    return -1;
  }

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSTATUS, at_cmode, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSTATUS, out, n));
}

/**
  Get response to GetStatus command

  Format:  +CIPSTATUS:<link ID>,<type>,<remote IP>,<remote port>,<local port>,<tetype>
  Example: +CIPSTATUS:0,"TCP","192.168.4.2",54600,80,1

  \param[out] conn         connection info

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_GetStatus (AT_DATA_LINK_CONN *conn) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Got valid argument */
      if (a == 0) {
        /* Read <link_id> (buf = integer) */
        uval = strtoul (p, &p, 10);

        conn->link_id = (uint8_t)uval;
      }
      else if (a == 1) {
        /* Read type string "UDP/TCP/SSL" */
        strcpy (conn->type, &p[1]);
      }
      else if (a == 2) {
        /* Read <remote_ip> (buf = "xxx.xxx.xxx.xxx") */
        AT_Parse_IP (p, conn->remote_ip);
      }
      else if (a == 3) {
        /* Read <remote_port> (buf = integer?) */
        uval = strtoul (p, &p, 10);

        conn->remote_port = (uint16_t)uval;
      }
      else if (a == 4) {
        /* Read <local_port> (buf = integer?) */
        uval = strtoul (p, &p, 10);

        conn->local_port = (uint16_t)uval;
      }
      else if (a == 5) {
        /* Read client/server flag */
        uval = strtoul (p, &p, 10);

        conn->c_s = (uint8_t)uval;
      }
      else {
        break;
      }

      /* Increment number of arguments */
      a++;
    }
  }
  while ((val != 2) && (val != 3));

  if (val == 3) {
    /* Last response */
    val = 0;
  }
  else {
    if (val == 2) {
      /* Response is pending */
      val = 1;
    }
  }

  return (val);
}


/**
  Resolve IP address from host name.

  Format S: AT+CIPDOMAIN=<domain name>

  Example S: AT+CIPDOMAIN="iot.espressif.cn"

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  domain      domain name string (www.xxx.com)
  
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_DnsFunction (uint32_t at_cmode, const char *domain) {
  char out[280];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPDOMAIN, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "\"%s\"", domain);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPDOMAIN, out, n));
}

/**
  Get response to DnsFunction command

  Format: +CIPDOMAIN:<IP address>

  Example: +CIPDOMAIN:192.168.4.2   (ESP)
  Example: +CIPDOMAIN:"192.168.4.2" (WIZ)

  \param[out] ip          IP address

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_DnsFunction (uint8_t ip[]) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to start of string */
      p = (char *)&buf[0];

      /* Parse IP address */
      AT_Parse_IP (p, ip);
      break;
    }
  }
  while (val != 2);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}


/**
  Establish TCP connection.

  Format S: AT+CIPSTART=<link ID>,<type>,<remote IP>,<remote port>[,<TCP keep alive>] (AT+CIPMUX=1)

  Example S: AT+CIPSTART=1,"TCP","192.168.1.100",8000

  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  link_id     ID of the connection
  \param[in]  ip          remote ip number
  \param[in]  port        remote port number
  \param[in]  keep_alive  TCP keep alive, 0:disable or [1:7200] in seconds to enable keep alive
  
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_ConnOpenTCP (uint32_t at_cmode, uint32_t link_id, const uint8_t ip[], uint16_t port, uint16_t keep_alive) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSTART, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d,\"%s\",\"%d.%d.%d.%d\",%d",
                           link_id,
                           "TCP",
                           ip[0], ip[1], ip[2], ip[3],
                           port);
    if (keep_alive != 0U) {
      n += sprintf (&out[n], ",%d", keep_alive);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSTART, out, n));
}


/**
  Establish UDP transmission.

  Format S: AT+CIPSTART=<link ID>,<type>,<remote IP>,<remote port>[,<UDP local port>, <UDP mode>] (AT+CIPMUX=1)

  Example S: AT+CIPSTART=1,"UDP","192.168.1.100",8000

  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  link_id     ID of the connection
  \param[in]  r_ip        remote ip number
  \param[in]  r_port      remote port number
  \param[in]  l_port      local port (ESP8266 rejects zero)
  \param[in]  mode        UDP mode (0:dst peer entity will not change, 1:will change once, 2:allowed to change)
  
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_ConnOpenUDP (uint32_t at_cmode, uint32_t link_id, const uint8_t r_ip[], uint16_t r_port, uint16_t l_port, uint32_t mode) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSTART, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d,\"%s\",\"%d.%d.%d.%d\",%d",
                           link_id,
                           "UDP",
                           r_ip[0], r_ip[1], r_ip[2], r_ip[3],
                           r_port);
    if (l_port != 0U) {
      /* Add optional arguments */
      n += sprintf (&out[n], ",%d,%d", l_port, mode);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSTART, out, n));
}



/**
  Close the TCP/UDP/SSL connection.

  Format S: AT+CIPCLOSE=<link_id>
  Format E: AT+CIPCLOSE
  
  Example: AT+CIPCLOSE=4;

  If ID of the connection is 5, all connections will be closed.
  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  link_id     ID of the connection

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_ConnectionClose (uint32_t at_cmode, uint32_t link_id) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPCLOSE, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", link_id);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPCLOSE, out, n));
}

/**
  Get ping response time.

  Command: PING
  Format:  AT+PING=<IP>
  Example: AT+PING="192.168.1.1"
  Example: AT+PING="www.xxx.com"

  Domain can be specified either by IP or by domain name string - only one should be
  specified.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  ip          IP address (xxx.xxx.xxx.xxx)
  \param[in]  domain      domain name string (www.xxx.com)
  
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_Ping (uint32_t at_cmode, const uint8_t ip[], const char *domain) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_PING, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    if (ip != NULL) {
      n += sprintf (&out[n], "\"%d.%d.%d.%d\"", ip[0], ip[1], ip[2], ip[3]);
    }
    if (domain != NULL) {
      n += sprintf (&out[n], "\"%s\"", domain);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_PING, out, n));
}

/**
  Get response to Ping command.
  
  Response: +<time>
  Response: +timeout
  Example:  +5

  \param[out] time        ping response time in ms

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data

  \return execution status
          - negative: error
          - 0: list is empty
          - 1: list contains more data
*/
int32_t AT_Resp_Ping (uint32_t *time) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      if ((p[0] >= '0') && (p[0] <= '9')) {
        /* Integer value */
        *time = strtoul (&p[0], &p, 10);
      } else {
        /* Got "timeout" string */
        *time |= 0x80000000U;
      }
      break;
    }
  }
  while (val != 2);

//  if (val < 0) {
//    val = -1;
//  } else {
    val = 0;
//  }

  return (val);
}


/**
  Set send data command.

  Command: CIPSEND
  Format:  AT+CIPSEND=<link_id>,<length>[,<remote IP>,<remote port>]

  \param[in]  at_cmode    command mode (inquiry, set, exec)
  \param[in]  link_id     connection id
  \param[in]  length      number of bytes to send
  \param[in]  remote_ip   remote IP (UDP transmission)
  \param[in]  remote_port remote port (UDP transmission)
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_SendData (uint32_t at_cmode, uint32_t link_id, uint32_t length, const uint8_t remote_ip[], uint16_t remote_port) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSEND, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d,%d", link_id, length);

    if (remote_ip != 0U) {
      /* Add optional arguments */
      n += sprintf (&out[n], ",\"%d.%d.%d.%d\",%d",
                              remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3],
                              remote_port);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSEND, out, n));
}


/**
  Set/Query connection type (single, multiple connections)

  Command: CIPMUX

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0:single connection, 1:multiple connections
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_ConnectionMux (uint32_t at_cmode, uint32_t mode) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPMUX, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", mode);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPMUX, out, n));
}

/**
  Get response to ConnectionMux command

  Response Q: +CIPMUX:<mode>
  Example  Q: +CIPMUX:1

  \param[out] mode      0:single connection, 1:multiple connections

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_ConnectionMux (uint32_t *mode) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      /* Read <mode> */
      *mode = p[0] - '0';
      break;
    }
  }
  while (val < 2);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}

/**
  Create or delete TCP server.

  Command: CIPSERVER
  Format:  AT+CIPSERVER=<mode>[,<port>]

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0:delete server, 1:create server
  \param[in]  port      port number
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_TcpServer (uint32_t at_cmode, uint32_t mode, uint16_t port) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSERVER, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", mode);
    
    if (port != 0U) {
      /* Add optional port number */
      n += sprintf (&out[n], ",%d", port);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSERVER, out, n));
}

/**
  Set the maximum connection allowed by server.

  Command: CIPSERVERMAXCONN
  Format:  AT+CIPSERVERMAXCONN=<num>

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  num       maximum number of clients allowed to connect
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_TcpServerMaxConn (uint32_t at_cmode, uint32_t num) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_CIPSERVERMAXCONN, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", num);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_CIPSERVERMAXCONN, out, n));
}

/**
  Get response to TcpServerMaxConn command

  Response Q: +CIPSERVERMAXCONN:<num>
  Example  Q: +CIPSERVERMAXCONN:1

  \param[in]  num       maximum number of clients allowed to connect

  \return execution status
          - negative: error
          - 0: OK, response retrieved, no more data
*/
int32_t AT_Resp_TcpServerMaxConn (uint32_t *num) {
  uint8_t buf[32];
  int32_t val;
  char *p;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      /* Read <num> */
      *num = p[0] - '0';
      break;
    }
  }
  while (val < 2);

  if (val < 0) {
    val = -1;
  } else {
    val = 0;
  }

  return (val);
}

/* ------------------------------------------------------------------------- */

/**
  Open AT command string (construct string: AT+<cmd><mode>)

  \param[in]  cmd_code  command code
  \param[in]  cmd_mode  command mode (AT_CMODE_QUERY, AT_CMODE_SET)
  \param[out] buf       AT command string output buffer
*/
static int32_t CmdOpen (uint8_t cmd_code, uint32_t cmd_mode, char *buf) {
  const char *Ctrl_AT = "AT+";
  const char *at;
  const char *cmd;
  char  chm;
  int32_t n;

  at  = Ctrl_AT;
  cmd = CmdString (cmd_code);

  if (cmd_mode == AT_CMODE_QUERY) {
    chm = '?';
  }
  else if (cmd_mode == AT_CMODE_SET) {
    chm = '=';
  }
  else {
    chm = '\0';
  }

  n = sprintf (buf, "%s%s%c", at, cmd, chm);

  if (chm == '\0') {
    n -= 1;
  }

  return (n);
}


/**
  Send AT command string (append crlf to command string and send)

  \param[in]  cmd   command code 
  \param[in]  buf   string buffer
  \param[in]  num   number of bytes from buf to send

  \return 0:OK, -1: error
*/
static int32_t CmdSend (uint8_t cmd, char *buf, int32_t num) {
  const char *Ctrl_CRLF = "\r\n";
  int32_t rval;
  int32_t sent;

  rval = -1;

  if (CmdSetWFE(cmd) == 0) {
    /* Command registered, append CRLF */
    num += sprintf (&buf[num], "%s", Ctrl_CRLF);

    /* Send out the command data */
    sent = Serial_SendBuf ((uint8_t *)buf, (uint32_t)num);

    if (sent == num) {
      rval = 0;
    }
  }

  return (rval);
}


/**
  Register command that waits for the response.
  
  \param[in]  cmd   Command code (see command list definition)
*/
static int32_t CmdSetWFE (uint8_t cmd) {

  /* Store last command sent */
  pCb->cmd_sent = cmd;

  return (0);//OK
}


/**
  Determine maximum number of bytes to be sent using AT_Send_Data.

  \return number of bytes
*/
uint32_t AT_Send_GetFree (void) {
  uint32_t cnt;

  cnt = Serial_GetTxFree();

  return (cnt);
}


/**
  Send data via currently active connection.

  \parma[in]  buf   data buffer
  \param[in]  len   number of bytes in buf to send

  \return number of bytes sent
*/
uint32_t AT_Send_Data (const uint8_t *buf, uint32_t len) {
  int32_t rval;
  uint32_t n;

  /* Send out the command data */
  rval = Serial_SendBuf (buf, len);
  
  if (rval < 0) {
    n = 0U;
  } else {
    n = (uint32_t)rval;
  }

  /* Return number of bytes actually sent */
  return (n);
}

/* ------------------------------------------------------------------------- */

/**
  Parse IP address from string to byte value.
*/
static void AT_Parse_IP (char *buf, uint8_t ip[]) {
  char *p;

  /* Set pointer to start of string */
  p = (char *)&buf[0];

  if (p[0] == '"') {
    /* Strip out the first quotation mark */
    p++;
  }

  /* Parse IP address (xxx.xxx.xxx.xxx or "xxx.xxx.xxx.xxx") */
  ip[0] = (uint8_t)strtoul (&p[0], &p, 10);
  ip[1] = (uint8_t)strtoul (&p[1], &p, 10);
  ip[2] = (uint8_t)strtoul (&p[1], &p, 10);
  ip[3] = (uint8_t)strtoul (&p[1], &p, 10);
}

/**
  Parse MAC address from (hex) string to byte value.
*/
static void AT_Parse_MAC (char *buf, uint8_t mac[]) {
  char *p;

  /* Set pointer to start of string */
  p = (char *)&buf[0];

  if (p[0] == '"') {
    /* Strip out the first quotation mark */
    p++;
  }

  /* Parse MAC address (xx:xx:xx:xx:xx:xx or "xx:xx:xx:xx:xx:xx") */
  mac[0] = (uint8_t)strtoul (&p[0], &p, 16);
  mac[1] = (uint8_t)strtoul (&p[1], &p, 16);
  mac[2] = (uint8_t)strtoul (&p[1], &p, 16);
  mac[3] = (uint8_t)strtoul (&p[1], &p, 16);
  mac[4] = (uint8_t)strtoul (&p[1], &p, 16);
  mac[5] = (uint8_t)strtoul (&p[1], &p, 16);
}
