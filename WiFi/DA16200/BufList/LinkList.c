/* -----------------------------------------------------------------------------
 * Copyright (c) 2019 Arm Limited (or its affiliates). All rights reserved.
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
 * $Date:        12. November 2019
 * $Revision:    V1.0
 *
 * Project:      Double linked list implementation
 * -------------------------------------------------------------------------- */

#include "LinkList.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/*
  Usage
  =====
  Add an element to the tail and remove it:

  typedef struct Chain {
    Link_t link;
    void *buf;
  } Chain;

  List_t list;
  Chain  buf;
  Chain *p;

  ListInit (&list);
  ListPut  (&list, (Link_t *)&buf);

  p = (Chain *)ListGet (&list);
*/

/**
  Initialize linked list

  \param[in]  list        Linked list pointer
 */
void ListInit (List_t *list) {

  list->head = NULL;
  list->tail = NULL;
}

/**
  Put a link at the end of a linked list.

  Used to implement FIFO linked list.

  \param[in]  list        Linked list pointer
  \param[in]  link        Link pointer
*/
void ListPut (List_t *list, Link_t *link) {
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  /* Atomic start */

  link->prev = list->tail;
  link->next = NULL;

  if (list->tail == NULL) {
    /* List empty, add head */
    list->head = link;
  }
  else {
    list->tail->next = link;
  }

  list->tail = link;

  /* Atomic end */
  if (primask == 0U) {
    __enable_irq();
  }
}

/**
  Put a link at the head of a linked list.

  Used to implement LIFO linked list.

  \param[in]  list        Linked list pointer
  \param[in]  link        Link pointer
*/
void ListPutHead (List_t *list, Link_t *link) {
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  /* Atomic start */

  link->prev = NULL;
  link->next = list->head;

  if (list->head == NULL) {
    /* List empty, add tail */
    list->tail = link;
  }
  else {
    list->head->prev = link;
  }

  list->head = link;

  /* Atomic end */
  if (primask == 0U) {
    __enable_irq();
  }
}

/**
  Get the first link in a linked list
  
  \param[in]  list        Linked list pointer
  \return Pointer to first link in the linked list, NULL if empty
*/
Link_t *ListGet (List_t *list) {
  uint32_t primask = __get_PRIMASK();
  Link_t *link;

  __disable_irq();
  /* Atomic start */

  link = list->head;

  if (link != NULL) {
    list->head = link->next;

    if (link->next == NULL) {
      /* List empty, clear tail */
      list->tail = NULL;
    }
    else {
      link->next->prev = NULL;
    }
  }

  /* Atomic end */
  if (primask == 0U) {
    __enable_irq();
  }

  return (link);
}

/**
  Retrieve head link (no remove)
  
  Can be used to check if list is empty: if (ListGetHead (list) == NULL) { isEmpty = true; }

  Traverse a list from head to tail:

  List_t *list;
  Link_t *link;

  for (link = ListPeekHead(list); link != NULL; link = ListPeekNext(link)) { ... }

  \param[in]  list        Linked list pointer
  \return Pointer to head link
*/
Link_t *ListPeekHead (List_t *list) {
  return (list->head);
}

/**
  Retrieve tail link (no remove)

  List_t  list;
  Link_t *link;

  for (link = ListPeekTail(list); link != NULL; link = ListPeekPrev(link)) { ... }

  \param[in]  list        Linked list pointer
  \return Pointer to tail link
*/
Link_t *ListPeekTail (List_t *list) {
  return (list->tail);
}

/**
  Retrieve next link (no remove)
  
  \param[in]  link        Pointer to list link
  \return Pointer to next link
*/
Link_t *ListPeekNext (Link_t *link) {
  return (link->next);
}

/**
  Retrieve previous link (no remove)
  
  \param[in]  link        Pointer to list link
  \return Pointer to next link
*/
Link_t *ListPeekPrev (Link_t *link) {
  return (link->prev);
}

/**
  Insert link in front of an existing link
  
  \param[in]  list        Linked list pointer
  \param[in]  link        Existing link
  \param[in]  lnew        Link to insert
*/
void ListInsert (List_t *list, Link_t *link, Link_t *lnew) {
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  /* Atomic start */

  lnew->next = link;
  lnew->prev = link->prev;

  if (link->prev == NULL) {
    /* No previous link, insert as head */
    list->head = lnew;
  }
  else {
    link->prev->next = lnew;
  }

  link->prev = lnew;

  /* Atomic end */
  if (primask == 0U) {
    __enable_irq();
  }
}

/**
  Remove link from the linked list

  \param[in]  list        Linked list pointer
  \param[in]  link        Link to remove
*/
void ListRemove (List_t *list, Link_t *link) {
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  /* Atomic start */

  if (link->next == NULL) {
    /* Last link in list */
    list->tail = link->prev;
  }
  else {
    link->next->prev = link->prev;
  }

  if (link->prev == NULL) {
    /* First link in list */
    list->head = link->next;
  }
  else {
    link->prev->next = link->next;
  }

  /* Atomic end */
  if (primask == 0U) {
    __enable_irq();
  }
}
