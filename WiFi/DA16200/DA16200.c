/* -----------------------------------------------------------------------------
 * Copyright (c) 2022 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        28. April 2022
 *
 * Project:      DA16200 WiFi Driver
 * -------------------------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "DA16200.h"
#include "DA16200_Serial.h"
#include "DA16200_DPM.h"
#include "WiFi_DA16200_Os.h"

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
  { "TRDTC"           },
  { "WFSCAN"          },
  { "WFJAPA"          },
  { "WFQAP"           },
  { "WFSAP"           },
  { "WFMODE"          },
  { "WFMAC"           },
  { "RFPOWER"         },
  { "NWIP"            },
  { "CIPDNS_CUR"      },
  { "NWDHC"           },
  { "NWDHS"           },
  { "WFDIS"           },
  { "CWLIF"           },
  { "ATB"             },
  { "TRPRT"           },
  { "NWHOST"          },
  { "TRTC"            },
  { "TRUR"            },
  { "TRTRM"           },
  { "TRTALL"          },
  { "NWPING"          },
  { "SENDTO"          },
  { "TRTS"            },
  { "RESTART"         },
  { "SDKVER"          },
  { "INIT"            },
  { "DPM"             },
  { "WFJAP"           },
  { "TRXTC"           },
  { "TRXTS"           },
  { "TRCTS"           },
  { "TRDTS"           },
  { "TRDUS"           },
  { "TRUSE"           },
  { "E"               },
  { ""                }
};

/* Command codes */
 typedef enum {
   CMD_TRDTC         = 0,
   CMD_WFSCAN,
   CMD_WFJAPA,
   CMD_WFQAP,
   CMD_WFSAP,
   CMD_WFMODE,
   CMD_WFMAC,
   CMD_RFPOWER,
   CMD_NWIP,
   CMD_CIPDNS_CUR,
   CMD_NWDHC,
   CMD_NWDHS,
   CMD_WFDIS,
   CMD_CWLIF,
   CMD_ATB,
   CMD_TRPRT,
   CMD_NWHOST,
   CMD_TRTC,                 /* TCP Client*/
   CMD_TRUR,                 /* UDP Client*/
   CMD_TRTRM,                /* close session by cid */
   CMD_TRTALL,               /* close all sessions */
   CMD_NWPING,
   CMD_SENDTO,
   CMD_TRTS,
   CMD_RESTART,
   CMD_SDKVER,
   CMD_INIT,
   CMD_DPM,
   CMD_WFJAP,
   CMD_TRXTC,
   CMD_TRXTS,
   CMD_TRCTS,
   CMD_TRDTS,
   CMD_TRDUS,
   CMD_TRUSE,
   CMD_ECHO          = 0xFD, /* Command Echo                 */
   CMD_TEST          = 0xFE, /* AT startup (empty command)   */
   CMD_UNKNOWN       = 0xFF  /* Unknown or unhandled command */
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

int tcp_link_id = -1;
int udp_link_id = -1;

char tcpc_session = 0;
char tcps_session = 0;
char udp_session = 0;


uint8_t r_ip[4];
uint16_t r_port;

uint8_t ur_ip[4];
uint16_t ur_port;
uint16_t ul_port;

uint32_t scan_len;

    
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
        if ((pCb->resp_code == CMD_TRDTC) || (pCb->resp_code == CMD_TRDTS) || (pCb->resp_code == CMD_TRDUS)) {
          /* Copy response (including ',' character) */
          BufCopy (&(pCb->resp), &(pCb->mem), pCb->resp_len+1);

          AT_DATA_LINK_CONN conn;
          extern uint8_t client_conn_check;
          if ((pCb->resp_code == CMD_TRDTC) &&   (client_conn_check == 0)) {
            conn.c_s = 0;               
            conn.link_id = tcp_link_id;
            memcpy(conn.remote_ip, r_ip, 4);
            conn.remote_port = r_port;
            strcpy(conn.type, "TCP");
            conn.local_port = 30000;

            AT_Notify (AT_NOTIFY_CONNECTION_OPEN, (void*)&conn);
          } else if ((pCb->resp_code == CMD_TRDUS) && (client_conn_check == 0)) {
            conn.c_s = 0;               
            conn.link_id = udp_link_id;
            memcpy(conn.remote_ip, ur_ip, 4);
            conn.remote_port = ur_port;
            strcpy(conn.type, "UDP");
            conn.local_port = ul_port;

            AT_Notify (AT_NOTIFY_CONNECTION_OPEN, (void*)&conn);
          }

          /* Receive network data (+IPD) */
          pCb->ipd_rx = 0U;

          AT_Notify (AT_NOTIFY_CONNECTION_RX_INIT, &(pCb->ipd_rx));

          if (pCb->ipd_rx == 0U) {
            /* Socket is out of memory */
            AT_Notify (AT_NOTIFY_OUT_OF_MEMORY, NULL);
          }

          /* Start receiving data */
          pCb->state = AT_STATE_RECV_DATA;
          if (pCb->ipd_rx == 0xFFFFFFFF) {
            /* Null Socket */
            pCb->state = AT_STATE_ANALYZE;
          }
        }
        else if (pCb->resp_code == CMD_WFSCAN){
          /* Copy response (including "\r\n" characters) */
          BufCopy (&(pCb->resp), &(pCb->mem), scan_len+2);
          pCb->state = AT_STATE_ANALYZE;
        }
        else {
          /* Response data arrived */
          /* Copy response (including "\r\n" characters) */
          BufCopy (&(pCb->resp), &(pCb->mem), pCb->resp_len+2);

          pCb->state = AT_STATE_ANALYZE;

          if (pCb->resp_code == CMD_TRXTC) {
            /* Client Disconnect from server*/
            AT_DATA_LINK_CONN conn;

            conn.link_id = tcp_link_id;            
            AT_Notify (AT_NOTIFY_CONNECTION_CLOSED, &conn);
            pCb->state = AT_STATE_FLUSH;
          } else if (pCb->resp_code == CMD_TRXTS) {
            /* Client Disconnect from TCP Server */
            AT_DATA_LINK_CONN conn;        
            conn.link_id = tcps_link_id;    
            AT_Notify (AT_NOTIFY_CONNECTION_CLOSED, &conn);
            pCb->state = AT_STATE_FLUSH;
          } else if (pCb->resp_code == CMD_INIT) {
            AT_Notify (AT_NOTIFY_WAKEUP, NULL);
          } else if (pCb->resp_code == CMD_TRCTS) {
            /* Connect TCP Server */
            AT_DATA_LINK_CONN conn;
            conn.c_s = 1;
            conn.link_id = tcps_link_id;
            conn.local_port = tcps_local_port;

            AT_Notify (AT_NOTIFY_CONNECTION_OPEN, &conn);
          } else if (pCb->resp_code != CMD_UNKNOWN) {
            /* Command response (+XXX in buffer) */
            pCb->state = AT_STATE_ANALYZE;          
          } else {
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
        /* Not terminated */  /* receive  data check and not enough  then re-check */
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

      if ((pCb->resp_code == CMD_TRDTC) || (pCb->resp_code == CMD_TRDTS) || (pCb->resp_code == CMD_TRDUS)) {
        /*format : +CMD:<cid>,<ip>,<port>,<length>,<data> */
        pCb->resp_len = (uint8_t)BufFindByteNth (',', 4, pMem);
    
        if (pCb->resp_len > 36) {
          /* +TRxxx:x,xxx.xxx.xxx.xxx,xxxxx,xxxx, */
          rval = AT_STATE_WAIT;
        }
        else {
          rval = AT_STATE_RESP_DATA;
        }
        
      } 
      else if(pCb->resp_code == CMD_WFSCAN){
        n = BufFind (crlf, 2, pMem);

        if (n == -1) {
          /* Not terminated, wait for more data */
          rval = AT_STATE_WAIT;
        }
        else {
          /* Line terminator found */

          scan_len = n;             
          rval = AT_STATE_RESP_DATA;
        }
      }  
      else
      {
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
      pCb->resp_code = CMD_NWPING;

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

  The return value should indicate continuation pattern. 

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

/**
  Get response argument.

  The return value should indicate continuation pattern. For example when there
  are multiple responses, as
  +WFSCAN:<bssid><\t><frequency><\t><rssi><\t><security><\t><ssid>\n ...

  the return value should indicate what follows after \r\n termination:
  - in case if '+' follows, there is another response to be processed
  - in case if "OK" follows, response was processed completely.

  Note that +IPD response format is different and there is no \r\n terminator.

  \return -2: failed, specified buffer too small (sz of buf)
          -1: response incomplete, rx buffer empty
           0: retrieved, last delimiter: '\t'
           1: retrieved, last delimiter: '\n'
           2: retrieved, last delimiter: '\n', response pending ('+')
          // 3: retrieved, last delimiter: '\r', last response ("OK")
*/
static int32_t GetRespArgScan (uint8_t *buf, uint32_t sz) {
  uint32_t i;   /* argument size    */
  uint32_t str; /* string indicator */
  int32_t val;
  uint8_t b;
  uint8_t d[] = {'\n'};
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
  Retrieve incomming data.

  Response: +TRDTC:<C_ID>,<remote IP>,<remote port>,<len>,<data>

  This response does not have CRLF terminator, <len> is the number of bytes in <data>.
  Also note that the format of +IPD is also different in how argument are provided.

  \param[out]  link_id     connection ID
  \param[out]  remote_ip   remote IP
  \param[out]  remote_port remote port
  \param[out]  len         data length


  \return 0: ok, len of data shall be read from buffer
         negative: buffer empty or packet incomplete
*/
int32_t AT_Resp_TRDTC (uint32_t *link_id, uint32_t *len, uint8_t *remote_ip, uint16_t *remote_port) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */
  uint8_t valid_check;
 
  a = 0U;
  valid_check = 0U;

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

      if (uval == TCP_S) {
        *link_id = tcps_link_id;
      }
      else if (uval == TCP_C) {
        *link_id = tcp_link_id;
      }
      else if (uval == UDP){
        *link_id = udp_link_id;
      }

      if(val != 0){
        valid_check ++;
      }
    }
    else if (a == 1) {
      if(val != 0){
        valid_check ++;
      }
    }
    else if (a == 2) {
      if(val != 0){
        valid_check ++;
      }
    }
    else if (a == 3) {
      /* Read <len> (buf = integer) */
      uval = strtoul (p, &p, 10);
      *len = uval;
      if(val != 0){
        valid_check ++;
      }

      if (valid_check == 0) {
        /* At the ',' delimiter */
        val = 0;
        break;
      }
    }
    else {
      /* ??? */
      break;
    }

    /* Increment number of arguments */
    if (valid_check > 0){
      a = 0;
      valid_check = 0;
    }else{
      a++;
    }
  }
  while (val >= 0);

  return (val);
}

/**
Test AT startup

Generic response is expected.

return 0:OK, -1: error
*/
int32_t AT_Cmd_TestAT (void) {
  char out[8];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "%s", "AT");

  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
}

/**
  Restarts the module

  Generic response is expected.

  Format: AT+RST

  return 0:OK, -1: error
*/
int32_t AT_Cmd_Reset (void) {
  char out[16];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_RESTART, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_RESTART, out, n));
}

/**
  Check version information

  Generic response is expected.

  Format: AT+SDKVER

  return 0:OK, -1: error
*/
int32_t AT_Cmd_GetVersion (void) {
  char out[16];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_SDKVER, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_SDKVER, out, n));
}

/**
  Get response to GetVersion command

  param[out] buf   data buffer
  param[in]  len   data buffer size
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

  if(enable)
    /* command echo */
    n = sprintf (out, "%s", "ATE");
  else
    /* echo off */
    n = sprintf (out, "%s", "ATZ");

  /* Append CRLF and send command */
  return (CmdSend(CMD_ECHO, out, n));
}

/**
  Set/Query the current UART configuration  but not Support
  Default set : baudrate:115200, data : 8 bit, stop: 1bit, parity : None, flow control: None

  Format S: AT+CMD_ATB=<baudrate>,<databits>,<parity>,<stopbits>,<flow control>
  Format Q: AT+CMD_ATB=?

  Example S: AT+CMD_ATB=115200,8,1,0,0\r\n

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

  if (at_cmode == AT_CMODE_SET) {
    stopbits  = (stop_par_flowc >> 4) & 0x3;
    parity    = (stop_par_flowc >> 2) & 0x3;
    flow_ctrl = (stop_par_flowc >> 0) & 0x3;

    /* Add command arguments */
    n = sprintf (out, "ATB=%d,%d,%d,%d,%d", baudrate, databits, parity, stopbits, flow_ctrl);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_ATB, out, n));
}


/**
  Get response to ConfigUART command

  Response Q: +BAUDRATE:<baudrate>\r\n\r\n\OK

  \param[out] baudrate
  \return
*/
int32_t AT_Resp_ConfigUART (uint32_t *baudrate) {
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t uval;  /* Unsigned value storage */


  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    /* Ignore ':' delimiter */
    if (val != -1) {
      p = (char *)&buf[0];

      /* Read <baudrate> */
      uval = strtoul (p, &p, 10);

      /* Note: if S was 115200, Q might return 115273 */
      *baudrate = uval;
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
  Set maximum value of RF TX power (dBm).

  NOTE : NOT SUPPORT

  Response: Generic

  \param[in]  tx_power  power value
  \return 0: ok, -1: error
*/
int32_t AT_Cmd_TxPower (uint32_t tx_power) {
  char out[32];
  int32_t n;

  return 0;

}

/**
  Set/Query the current Wi-Fi mode

  Format S: AT+WFMODE=<mode>

  Response Q: AT_Resp_CurrentMode

  NOTE : SUPPORT only  Station Mode

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      Mode, 0: station, 1: soft AP
  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Cmd_CurrentMode (uint32_t at_cmode, uint32_t mode) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_WFMODE, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", mode);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_WFMODE, out, n));
}

/**
  Get response to CurrentMode command

  NOTE : SUPPORT only  Station Mode

  Response Q: +WFMODE:<mode>
  Example  Q: +WFMODE:0\r\n\r\n\OK
  \param[in]  mode      Mode, 0: station, 1: soft AP

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
  Set/Query connected access point or access point to connect to

  Format S: AT+WFJAPA=<ssid>,<pwd>
  Format Q: AT+WFJAPA=?

  Response Q: AT_Resp_ConnectAP

  \param[in]  ssid
  \param[in]  pwd
  \return 0: ok, -1: error
*/
int32_t AT_Cmd_ConnectAP (uint32_t at_cmode, const char *ssid, const char *pwd, const char *bssid) {
  char out[64];
  int32_t n, rval;

    if (at_cmode == AT_CMODE_SET) {
      /* Open AT command (AT+<cmd><mode> */
      n = CmdOpen (CMD_WFJAPA, at_cmode, out);

      /* Add command arguments */
      n += sprintf (&out[n], "%s,%s", ssid, pwd);

      /* Append CRLF and send command */
      rval = CmdSend(CMD_WFJAPA, out, n);
    }
    else if (at_cmode == AT_CMODE_QUERY) {
      /* Open AT command (AT+<cmd><mode> */
      n = CmdOpen (CMD_WFJAP, at_cmode, out);

      /* Append CRLF and send command */
      rval = CmdSend(CMD_WFJAP, out, n);
    }

    return rval;
}

/**
  Response to ConnectAP command.

  Response Q: +WFJAPA=?
  Example  Q: +WFJAPA:<AP_SSID>,<passphase>

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
        /* Read <password> (buf = "string") */
        strcpy (ap->pwd, (const char *)buf);
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
  Disconnect from current Access Point (WFQAP)

  \return 0:ok, -1: error
*/
int32_t AT_Cmd_DisconnectAP (void) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_WFQAP, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_WFQAP, out, n));
}


/**
  Configure local access point (SoftAP must be active)

  NOTE : NOT SUPPORT NOW

  sec: 0=OPEN, 2=WPA, 3=WPA2, 4=WPA+WPA2, 5=WPA3_OWE, 6=WPA3_SAE, 7=WPA2+WPA3
  enc: 0=TKIP, 1=AES, 2=TKIP+AES

  Format: AT+WFSAP=<ssid>,<sec>,<ecn>,<pwd>[,<ch>][,<country>]
*/
int32_t AT_Cmd_ConfigureAP (uint32_t at_cmode, AT_DATA_CWSAP *cfg) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_WFSAP, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%s,%d,%d,%s", cfg->ssid, cfg->sec, cfg->ecn, cfg->pwd);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_WFSAP, out, n));
}

/**
  Get response to ConfigureAP command

  NOTE : NOT SUPPORT NOW

  Format: +WFSAP:<ssid>,<sec>,<ecn>,<ssid>,<ch>,<country>
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
        /* Read <sec> */
        cfg->sec = p[0] - '0';
      }
      else if (a == 2) {
        /* Read <ecn> */
        cfg->ecn = p[0] - '0';
      }
      else if (a == 3) {
        /* Read <pwd> */
        strcpy (cfg->pwd, (const char *)buf);
      }      
      else if (a == 4) {
        /* Read <ch> */
        uval = strtoul (p, &p, 10);

        cfg->ch = (uint8_t)uval;
      }
      else if (a == 5) {
        /* Read <country> */
        strcpy (cfg->country, (const char *)buf);
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
  List available Access Points (WFSCAN)
*/
int32_t AT_Cmd_ListAP (void) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_WFSCAN, AT_CMODE_EXEC, out);

  /* Append CRLF and send command */
  return (CmdSend(CMD_WFSCAN, out, n));
}

/**
  Process input string

  Format: +WFSCAN:<bssid><frequency><rssi><security><ssid>

  \return execution status
          - negative: error
          - 0: access point list is empty
          - 1: access point list contains more data
*/
int32_t AT_Resp_ListAP (AT_DATA_CWLAP ap[], uint8_t max_num) {
  char    *p;
  uint8_t  buf[128]= {0,};
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */
  uint8_t i;
  char* list_buf[5];        /* 0:bssid, 1:freq, 2:ch, 3:security string, 4:ssid */
  
  a = 0U;
  i = 0;
  do {
    /* Retrieve response argument */    
    val = GetRespArgScan (buf, sizeof(buf));

    if (val < 0) {
      break;
    } 

    p = (char *)&buf[0];

    list_buf[0] = strtok(p, "\t");
    AT_Parse_MAC (list_buf[0], ap[i].mac);

    list_buf[1] = strtok(NULL, "\t");
    ap[i].freq_offs = atoi(list_buf[1]);
        
    list_buf[2] = strtok(NULL, "\t");
    ap[i].rssi = atoi(list_buf[2]);

    list_buf[3] = strtok(NULL, "\t");
    if (strstr((const char *)list_buf[3], "WPA") != 0) {
      if (strstr((const char *)list_buf[3], "EAP") != 0) {
        ap[i].ecn = AT_DATA_ECN_WPA2_E;
      }
      else if (strstr((const char *)list_buf[3], "WPA2-") != 0) {
        if (strstr((const char *)list_buf[3], "+SAE") != 0) {
          ap[i].ecn = AT_DATA_ECN_WPA2_E;
        }
        else if (strstr((const char *)list_buf[3], "-SAE") != 0) {
          if (strstr((const char *)list_buf[3], "-SHA256") != 0) {
            ap[i].ecn = AT_DATA_ECN_WPA2_E;
          }
          else {
            ap[i].ecn = AT_DATA_ECN_WPA2_E;
          }
        }
        else if (strstr((const char *)list_buf[3], "+OWE") != 0) {
          ap[i].ecn = AT_DATA_ECN_WPA2_E;
        }
        else if (strstr((const char *)list_buf[3], "-OWE") != 0) {
          ap[i].ecn = AT_DATA_ECN_WPA2_E;
        }
        else if (strstr((const char *)list_buf[3], "WPA-") != 0) {
          if (strstr((const char *)list_buf[3], "-SHA256") != 0) {
            /* PMF Required */
            ap[i].ecn = AT_DATA_ECN_WPA2_E;
          }
          else {
            ap[i].ecn = AT_DATA_ECN_WPA_WPA2_PSK;
          }
        }
        else {
          if (strstr((const char *)list_buf[3], "-SHA256") != 0) {
            ap[i].ecn = AT_DATA_ECN_WPA2_E;
          }
          else {
            ap[i].ecn = AT_DATA_ECN_WPA2_PSK;
          }
        }
      }
      else {
        ap[i].ecn = AT_DATA_ECN_WPA_PSK;
      }
    } 
    else if (strstr((const char *)list_buf[3], "WEP") != 0) {
      ap[i].ecn = AT_DATA_ECN_WEP;
    }
    else if (strstr((const char *)list_buf[3], "MESH") != 0) {
      if (strstr((const char *)list_buf[3], "RSN-SAE-CCMP") != 0) {
        ap[i].ecn = AT_DATA_ECN_WPA2_E;
      }
      else {                       
        ap[i].ecn = AT_DATA_ECN_WPA2_E;
      }
    }
    else {
      ap[i].ecn = AT_DATA_ECN_OPEN;
    }

    list_buf[4] = strtok(NULL, "\t");
    strcpy(ap[i].ssid, list_buf[4]);

    i++;
  }
  while(i < max_num );

  if (val < 0) {
    val = 0;
  }

  return (val);
}


/**
  Set station MAC address

  Do not set the same MAC address for Station and SoftAP.

  Format: AT+WFMAC=<mac>

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mac       Pointer to 6 byte array containing MAC address
*/
int32_t AT_Cmd_StationMAC (uint32_t at_cmode, const uint8_t mac[]) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_WFMAC, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%02X:%02X:%02X:%02X:%02X:%02X",
                          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_WFMAC, out, n));
}

/**
  Get response to StationMAC command

  String form: +WFMAC:<mac>\r\n\r\nOK

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

    NOTE : NOT SUPPORT NOW

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mac       Pointer to 6 byte array containing MAC address
  \return
*/
int32_t AT_Cmd_AccessPointMAC (uint32_t at_cmode, uint8_t mac[]) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_WFMAC, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%02X:%02X:%02X:%02X:%02X:%02X",
                            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_WFMAC, out, n));
}

/**
  Get response to AccessPointMAC command

    NOTE : NOT SUPPORT NOW

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

  Format S: AT+NWIP=<iface>,<ip>,<netmask>,<gateway>
  Format Q: AT+NWIP=?

  Example: AT+CIPSTA_CUR=0,192.168.1.100,255.255.255.0,192.168.6.1

  iface: 0=station, 1=SoftAP

  Response: AT_Resp_StationIP
*/
int32_t AT_Cmd_StationIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]) {
  char out[70];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_NWIP, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "0,%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    if (mask != NULL) {
      /* Add netmask */
      n += sprintf (&out[n], ",%d.%d.%d.%d", mask[0], mask[1], mask[2], mask[3]);

      if (gw != NULL) {
        /* Add gateway */
        n += sprintf (&out[n], ",%d.%d.%d.%d", gw[0], gw[1], gw[2], gw[3]);
      }
    }
  }

  return (CmdSend(CMD_NWIP, out, n));
}

/**
  Get response to StationIP command

  Response: +NWIP:<iface><ip><netmask><gateway>
  Example:  +NWIP:0,192.168.0.100,255.255.255.0,192.168.0.1

  \param[out]   ip          Pointer to 4 byte array where the address is stored
  \param[out]   netmask    Pointer to 4 byte array where the address is stored
  \param[out]   gateway    Pointer to 4 byte array where the address is stored
  \return execution status
          - negative: error
          - 0: address list is empty
          - 1: address list contains more data
*/
int32_t AT_Resp_StationIP (uint8_t ip_addr[], uint8_t nm_addr[],uint8_t gw_addr[]) {
  uint8_t  buf[32]; /* Argument buffer */
  int32_t  val;     /* Control value */
  char *p;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */
  
  a= 0U;

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

      if (a == 0) {    
        /* iface */
        /* uval = strtoul (p, &p, 10); */
        uval = p[0] - '0';
      }
      else  if (a == 1) {
        /* Parse IP address (xxx.xxx.xxx.xxx) */
        AT_Parse_IP (p, ip_addr);
      }
      else  if (a == 2) {
        /* Parse IP address (xxx.xxx.xxx.xxx) */
        AT_Parse_IP (p, nm_addr);
      }  
      else  if (a == 3) {
        /* Parse IP address (xxx.xxx.xxx.xxx) */
        AT_Parse_IP (p, gw_addr);
      }
      else {
        /* Unknown arguments */
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
  Set/Query current IP address of the local access point

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  iface    Station: 0 or AP: 1
  \param[in]  ip    IP address
  \param[in]  mask  Netmask
  \param[in]  gw    Gateway address
  \return
*/
int32_t AT_Cmd_AccessPointIP (uint32_t at_cmode, uint8_t ip[], uint8_t gw[], uint8_t mask[]) {
  char out[70];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_NWIP, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "1, %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    if (mask != NULL) {
      /* Add netmask */
      n += sprintf (&out[n], ",%d.%d.%d.%d", mask[0], mask[1], mask[2], mask[3]);

      if (gw != NULL) {
        /* Add gateway */
        n += sprintf (&out[n], ",%d.%d.%d.%d", gw[0], gw[1], gw[2], gw[3]);
      }
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_NWIP, out, n));
}


int32_t AT_Resp_AccessPointIP (uint8_t ip_addr[], uint8_t nm_addr[],uint8_t gw_addr[]) {
  return AT_Resp_StationIP (ip_addr, nm_addr, gw_addr);
}


/**
  Set user defined DNS servers

  NOTE : NOT SUPPORT NOW
*/
int32_t AT_Cmd_DNS (uint32_t at_cmode, uint32_t enable, uint8_t dns0[], uint8_t dns1[]) {
  char out[64];
  int32_t n;

  if (at_cmode == AT_CMODE_SET) {

  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
}


/**
  Get response to DNS command

  NOTE : NOT SUPPORT NOW

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

/**
  Set/Query DHCP state

  Format S: AT+NWDHC=<en>
  Format Q: AT+NWDHC=?

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable    0: stop DHCP, 1: start DHCP
*/
int32_t AT_Cmd_DHCP (uint32_t at_cmode, uint32_t mode, uint32_t enable) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_NWDHC, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d", enable);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_NWDHC, out, n));
}

/**
  Get response to DHCP command

  Response Q: +NWDHC:<enable>
  Example  Q: +NWDHC:1\r\n\r\n\OK

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

  NOTE : NOT SUPPORT NOW

  AT command is enabled when device runs as SoftAP, and when DHCP is enabled.
  The IP address should be in the same network segment as the IP address of device
  SoftAP. SoftAP IP address must be different than <start IP> or <end IP>.

  Format S: AT+NWDHS=<dhcpd>,<start IP>,<end IP>,<lease time>
  Format Q: AT+NWDHS=?

  Example: AT+NWDHS=1,192.168.4.10,192.168.4.15,1800

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  dhcpd    0 : stop, 1: start
  \param[in]  ip_start    first IP in range that can be obtained from DHCP server
  \param[in]  ip_end      last IP in range that can be obtained from DHCP server
  \param[in]  lease time   default 1800 sec
*/
int32_t AT_Cmd_RangeDHCP (uint32_t at_cmode, uint32_t en_tlease, uint8_t ip_start[], uint8_t ip_end[]) {
  char out[64];
  int32_t n;
  uint32_t en      = en_tlease >> 16;
  uint32_t t_lease = en_tlease & 0xFFFF;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_NWDHS, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    if (en != 0) {
      n += sprintf (&out[n], "%d,%d.%d.%d.%d,%d.%d.%d.%d,%d",
                             en,
                             ip_start[0], ip_start[1], ip_start[2], ip_start[3],
                             ip_end[0],   ip_end[1],   ip_end[2],   ip_end[3],
                             t_lease);
    }
    else {
      /* AT+CWDHCPS_CUR=0 */
      n += sprintf (&out[n], "%d", en);
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_NWDHS, out, n));
}

/**
  Get response to RangeDHCP command

  Response Q: +NWDHS=<dhcpd>
  Example  Q: +NWDHS=?

  NOTE : NOT SUPPORT NOW

  \param[out]  dhcpd  0 : stop, 1: start


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
        /* Read <dhcpd> (buf = integer) */
        uval = strtoul (&p[0], &p, 10);
      }
      else if (a == 1) {
        /* Read <start IP> (buf = "xxx.xxx.xxx.xxx") */
        AT_Parse_IP (p, ip_start);
      }
      else if (a == 2) {
        /* Read <end IP> (buf = "xxx.xxx.xxx.xxx") */
        AT_Parse_IP (p, ip_end);
      }
      else if (a == 3) {
        /* Read <lease time> (buf = integer) */
        uval = strtoul (&p[0], &p, 10);

        *t_lease = uval;
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

  Format: AT+WFDIS=<enable>

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  enable    0:enable auto-connect on power-up, 1:disable

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_AutoConnectAP (uint32_t at_cmode, uint32_t enable) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_WFDIS, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command argument */
    n += sprintf (&out[n], "%d", enable);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_WFDIS, out, n));


}

/**
  Get response to AutoConnectAP command

  Response Q: +WFDIS:<enable>
  Example  Q: +WFDIS:0\r\n\OK\r\n\

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


/* TCP/IP Related AT Commands ---------------------- */

/**
  Get the connection status.

  Format: AT+TRPRT

  \param[in] cid         connection info  0: TCP SERVER, 1: TCP Client, 2: UDP

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
  n = CmdOpen (CMD_TRPRT, at_cmode, out);

  /* Add command argument */
  if (tcps_session){
    n += sprintf (&out[n], "%d", 0);
  }
  else if (tcpc_session)   {
    n += sprintf (&out[n], "%d", 1);
  }
  else if (udp_session) {
    n += sprintf (&out[n], "%d", 2);
  }
  /* Append CRLF and send command */
  return (CmdSend(CMD_TRPRT, out, n));

}

/**
  Get response to GetStatus command

  Format:  +TRPRT:<cid>,<type>,<remote IP>,<remote port>,<local port>
  Example: +TRPRT:1,"TCP","192.168.4.2",54600,80

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

  Format S: AT+NWHOST=<domain name>

  Example S: AT+NWHOST=www.renesas.com

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  domain      domain name string (www.xxx.com)

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_DnsFunction (uint32_t at_cmode, const char *domain) {
  char out[280];
  int32_t n;


  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_NWHOST, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%s", domain);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_NWHOST, out, n));

}

/**
  Get response to DnsFunction command

  Format: +NWHOST:<IP address>

  Example: +NWHOST:192.168.4.2

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

  Format S: AT+TRTC=<remote IP>,<remote port>
  Example S: AT+TRTC=192.168.1.100,8000

  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  ip          remote ip number
  \param[in]  port        remote port number

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_ConnOpenTCP (uint32_t at_cmode, uint32_t link_id, const uint8_t ip[], uint16_t port, uint16_t keep_alive) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_TRTC, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d.%d.%d.%d,%d",
                           ip[0], ip[1], ip[2], ip[3],
                           port);
  }

  tcp_link_id = link_id; 
  tcpc_session = 1;
  r_port = port;
  memcpy (r_ip,ip, 4);

  /* Append CRLF and send command */
  return (CmdSend(CMD_TRTC, out, n));

}

/**
  Establish UDP transmission.

  Format S: AT+TRUR=<remote IP>,<remote port>
  Example S: AT+TRUR=192.168.1.100,8000

  Generic response is expected.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)

  \param[in]  r_ip        remote ip number
  \param[in]  r_port      remote port number

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_ConnUDP (uint32_t at_cmode, uint32_t link_id, const uint8_t r_ip[], uint16_t r_port, uint16_t l_port, uint32_t mode) {
  char out[64];
  int32_t n;
  int32_t ex, rval;


  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_TRUR, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d.%d.%d.%d,%d",
     	                     r_ip[0], r_ip[1], r_ip[2], r_ip[3],
                           r_port == 0? 1234 : r_port);    // not support r_port 0
  }

  udp_link_id = link_id;   
  udp_session = 1;
  ur_port = r_port;
  ul_port = l_port;
  memcpy (ur_ip,r_ip, 4);

  /* Append CRLF and send command */
  return (CmdSend(CMD_TRUR, out, n));

}

/**
  Establish UDP transmission.

  Format S: AT+TRUSE=<local port>
  Example S: AT+TRUSE=8000

  Generic response is expected.

  \param[in]  mode     mode (0: close if error response,  1: run )

  \param[in]  l_port      local port number

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_OpenUDP (uint32_t at_cmode,  uint16_t l_port) {
  char out[64];
  int32_t n, rval;
  
  if (at_cmode == AT_CMODE_SET){
    n = CmdOpen (CMD_TRUSE, at_cmode, out);
    n += sprintf (&out[n], "%d", l_port);
    rval = CmdSend(CMD_TRUSE, out, n);
  }
  else {
    n = CmdOpen (CMD_TRTRM, at_cmode, out);
    n += sprintf (&out[n], "%d", 2);
    rval = CmdSend(CMD_TRTRM, out, n);
  }

  return rval;
}


/**
  Close the TCP/UDP/SSL connection.

  Format S: AT+TRTRM=<c_id>
  Example: AT+TRTRM=1;  // for TCP Client

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  c_id     ID of the connection 0: TCP Server, 1: TCP Client, 2: UDP Session

  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_ConnectionClose (uint32_t at_cmode, uint32_t link_id) {
  char out[32];
  int32_t n;
 
  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_TRTRM, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    if (tcps_link_id == link_id) {
      return -1;
    }
    else if (tcp_link_id == link_id) {
      n += sprintf (&out[n], "%d", 1);
      tcpc_session = 0;
    }
    else if (udp_link_id == link_id) {
      n += sprintf (&out[n], "%d", 2);
      udp_session = 0;
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_TRTRM, out, n));
}

/**
  Get ping response time.

  Command: NWPING
  Format:  AT+NWPING=<iface><ip><count>
  Example: AT+NWPING=0,192.168.1.1,4

  Domain can be specified either by IP or by domain name string - only one should be
  specified.

  \param[in]  at_cmode    Command mode (inquiry, set, exec)
  \param[in]  iface       0:WLAN0, 1:WLAN1
  \param[in]  ip          IP address (xxx.xxx.xxx.xxx)
  \param[in]  count       The number of ICMP message transmissions


  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_Ping (uint32_t at_cmode, const uint8_t ip[], const char *domain) {
  char out[64];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_NWPING, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    if (ip != NULL) {
      n += sprintf (&out[n], "0,%d.%d.%d.%d,4", ip[0], ip[1], ip[2], ip[3]);  /* iface 0=WLAN0, default count 4 */
    }
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_NWPING, out, n));
}

/**
  Get response to Ping command.

  Response: +NWPING:<sent_count>,<recv_count>,<avg_time>,<min_time>,<max_time>
  Example:  +NWPING:4,4,0,0,0

  \param[out] sent_count
  \param[out] recv_count
  \param[out] avg_time
  \param[out] min_time
  \param[out] max_time

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
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */

  a = 0U;

  do {
    /* Retrieve response argument */
    val = GetRespArg (buf, sizeof(buf));

    if (val < 0) {
      break;
    }

    if (val != 1) {
      /* Set pointer to extracted value */
      p = (char *)&buf[0];

      if (a == 0) {
        /* Read send count */
        uval = strtoul (p, &p, 10);
      }
      else if (a == 1){
        /* Read receive count */
        uval = strtoul (p, &p, 10);
      }
      else if (a == 2){
        /* Read average time */
        *time = strtoul (p, &p, 10);
      }
      else if (a == 3){
        /* Read min_time */
        uval = strtoul (p, &p, 10);
      }
      else if (a == 4){
        /* Read max_time */
        uval = strtoul (p, &p, 10);
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
  val = 0;
  return (val);
}


/**
  Create or delete TCP server.

  Command: TRTS
  Format:  AT+TRTS=<port>


  Format: mode 1 :  AT+TRTS=<port>
                mode 0 :  AT+TRTRM=0

  Generic response is expected.

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      0:delete server, 1:create server
  \param[in]  port      port number
  \return execution status:
          0: OK, -1 on error
*/
int32_t AT_Cmd_TcpServer (uint32_t at_cmode, uint32_t mode, uint16_t port) {
  char out[32];
  int32_t n, rval;

  /* Open AT command (AT+<cmd><mode> */
  if (mode) {      // create server        
    n = CmdOpen (CMD_TRTS, at_cmode, out);

    if (at_cmode == AT_CMODE_SET) {
      /* Add command arguments */
      if (port != 0U) {
        /* Add optional port number */
        n += sprintf (&out[n], "%d", port);
      }
    }

    tcps_session = 1;
    /* Append CRLF and send command */
    rval = CmdSend(CMD_TRTS, out, n);
  }
  else {   // delete server        
    n = CmdOpen (CMD_TRTRM, at_cmode, out);
    tcps_session = 0;
    n += sprintf (&out[n], "%d", 0);
    /* Append CRLF and send command */        
    rval = CmdSend(CMD_TRTRM, out, n);
  }

  return rval;
}


/**
  Retrieve incomming data.

  Response: +TRCTS:<C_ID>,<remote IP>,<remote port>

   \param[out]  remote_ip   remote IP
   \param[out]  remote_port remote port


  \return 0: ok, len of data shall be read from buffer
          negative: buffer empty or packet incomplete
*/

int32_t AT_Resp_TcpServer (uint8_t ip_addr[], uint16_t *port) {
  uint8_t  buf[32]; /* Argument buffer */
  int32_t  val;     /* Control value */
  char *p; 
  uint32_t a;       /* Argument counter */
  uint32_t uval;    /* Unsigned value storage */

  a= 0U;

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

      if (a == 0) {	  
        /* link cid */
        uval = strtoul (p, &p, 10);
        /*uval = p[0] - '0'; */
      }
      else  if (a == 1) {
        /* Parse IP address (xxx.xxx.xxx.xxx) */
        AT_Parse_IP (p, ip_addr);
      }
      else  if (a == 2) {
        /* Read <remote_port> (buf = integer?) */
        *port = strtoul (p, &p, 10);
      }			 
      else {
        /* Unknown arguments */
      }

      /* Increment number of arguments */
      if (val == 2) {
        a = 0;
      }   
      else {
        a++;
      }
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
  Send data (reply to data transmit request).
*/
#define SERIAL_TXBUF_SZ   2048
uint32_t AT_Send_Data (char type, const uint8_t *buf, uint32_t len) {
  char out[SERIAL_TXBUF_SZ + 32] = {0,};      /* MAX DATA  + Header(<ESC>S<cid><len>,<ip>,<port>,<mode>, */
  int32_t n, ret;   

  out[0] = 0x1B;

  if (type == TCP_C)  {   
    n = sprintf (&out[1], "S%d%d,%d.%d.%d.%d,%d,r,",
            type,
            len,
            r_ip[0], r_ip[1], r_ip[2], r_ip[3],
            r_port);
  } else if (type == UDP) {
    n = sprintf (&out[1], "S%d%d,%d.%d.%d.%d,%d,r,",
            type,
            len,
            ur_ip[0], ur_ip[1], ur_ip[2], ur_ip[3],
            ur_port);
  }

  n +=1;
  memcpy(&out[n], buf, len);
  ret = Serial_SendBuf ((uint8_t *)out, (uint32_t)n+len);
  return (ret - n);
}


/**
  Check DPM Wake Up Type
*/
int32_t AT_Resp_WakeUp(uint8_t *type)
{
  char    *p;
  uint8_t  buf[32];
  int32_t  val;
  uint32_t a;     /* Argument counter */
  uint32_t uval;  /* Unsigned value storage */
  uint8_t  wakeup;

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
        if (strcasecmp((const char *)buf, "WAKEUP") == 0) {
          wakeup = 1;
        }
        else if (strcasecmp((const char *)buf, "DONE") == 0) {
          wakeup = 0;
        }
      }
      else if (a == 1) {
        if (wakeup) {
          if (strcasecmp((const char *)buf, "UC") == 0){
            uval = 2;
          }
          else  if (strcasecmp((const char *)buf, "NOBCN") == 0) {
            uval = 3;
          }
          else if (strcasecmp((const char *)buf, "DEAUTH") == 0) {
            uval = 4;
          }
          else  if (strcasecmp((const char *)buf, "EXT") == 0) {
            uval = 5;
          }
          else  if (strcasecmp((const char *)buf, "RTC") == 0) {
            uval = 6;
          }
        }
        else {
          /* Read <mode> (buf = integer) */
          uval= strtoul (p, &p, 10);    /* uval = 0 for station  */
        }

        *type = uval;
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
  Set/Query the DPM mode

  Format S: AT+DPM=<mode>

  Response Q: AT_Resp_DPM

  \param[in]  at_cmode  Command mode (inquiry, set, exec)
  \param[in]  mode      Mode, 0: Off, 1: On
  \return 0: OK, -1: error (invalid mode, etc)
*/

int32_t AT_Cmd_DPM (uint32_t at_cmode, uint8_t mode, uint8_t nvm) {
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = CmdOpen (CMD_DPM, at_cmode, out);

  if (at_cmode == AT_CMODE_SET) {
    /* Add command arguments */
    n += sprintf (&out[n], "%d,%d", mode, nvm);
  }

  /* Append CRLF and send command */
  return (CmdSend(CMD_DPM, out, n));
}

/**
  Get response to DPM command

  Response Q: +DPM:<mode>
  Example  Q: +DPM:0\r\n\r\n\OK
  \param[in]  mode      Mode, 0: Off, 1: On

  \return 0: OK, -1: error (invalid mode, etc)
*/
int32_t AT_Resp_DPM (uint32_t *mode) {
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
  Set the user application not to enter the DPM sleep

  Format S: AT+CLRDPMSLPEXT

  Response Q: AT_Resp_Gen

  \return 0: OK, -1: error (invalid mode, etc)
*/

int32_t AT_Cmd_ClrDPMSleep(void){
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "AT+CLRDPMSLPEXT");

  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
}

/**
  Set the user application ready to enter the DPM sleep

  Format S: AT+SETDPMSLPEXT=<sleep_type>,<period>,<retain>

  Response Q: AT_Resp_Gen

  DPM Sleep 1 can be woken up by RTC_WAKE_UP or GPIO which has been assigned as a wake-up source.
  DPM Sleep 2 can be woken up by RTC_WAKE_UP but RTC timer is one time.
  DPM Sleep 3 can be woken up by an external wake-up signal.

  \param[in]  sleep_type      1: sleep 1 mode, 2: sleep 2 mode  default :sleep 3 mode
  \param[in]  period          wake up time in second
  \param[in]  retain          1: retain,    2: not retain

  \return 0: OK, -1: error (invalid mode, etc)
*/

int32_t AT_Cmd_SetDPMSleep(uint8_t sleep_type, uint32_t period, uint8_t retain ){
  char out[32];
  int32_t n;

  switch(sleep_type) {
    case 1:
      /* enter DPM Sleep 1 mode */
      n = sprintf (out, "AT+SETSLEEP1EXT=%d",retain);
      break;
    case 2:
      /* enter DPM Sleep 2 mode for the period*/
      n = sprintf (out, "AT+SETSLEEP2EXT=%d,%d",period,retain);
      break;
    case 3:
      /* enter DPM Sleep 3 mode */
      n = sprintf (out, "AT+SETDPMSLPEXT");
      break;
  }
  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
}

/**
  Set DPM keepalive period

  Format S: AT+DPMKA=<keepalive>

  Response Q: AT_Resp_Gen

  \param[in]  keepalive      0 ~ 600000 millisecond
  \return 0: OK, -1: error (invalid mode, etc)
*/

int32_t AT_Cmd_DPMKA(uint32_t keepalive){
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "AT+DPMKA=%u", keepalive);

  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
}

/**
  Set DPM TIM wake-up count

  Format S: AT+DPMTIMWU=<count>

  Response Q: AT_Resp_Gen

  \param[in]  count      1 ~ 65535 count
  \return 0: OK, -1: error (invalid mode, etc)
*/

int32_t AT_Cmd_DPMTW(uint32_t count){
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "AT+DPMTIMWU=%u", count);

  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
}


/**
  Set DPM user wake-up time :

  Format S: AT+DPMUSERWU=<time>

  Response Q: AT_Resp_Gen

  \param[in]  time      0 ~ 86400 sec
  \return 0: OK, -1: error (invalid mode, etc)
*/

int32_t AT_Cmd_DPMUW(uint32_t time){
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "AT+DPMUSERWU=%u", time);

  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
}


/**
  Notify MCU wake-up done if MCU woken by external

  Format S: AT+MCUWUDONE

  Response Q: AT_Resp_Gen

  \return 0: OK, -1: error (invalid mode, etc)
*/

int32_t AT_Cmd_MCUWUDone(void){
  char out[32];
  int32_t n;

  /* Open AT command (AT+<cmd><mode> */
  n = sprintf (out, "AT+MCUWUDONE");

  /* Append CRLF and send command */
  return (CmdSend(CMD_TEST, out, n));
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
		chm = NULL;
  }
  else {
    chm = '\0';
  }

  n = sprintf (buf, "%s%s=%c", at, cmd, chm);

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
