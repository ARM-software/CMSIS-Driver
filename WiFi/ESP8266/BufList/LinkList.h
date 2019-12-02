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

#ifndef LINKLIST_H__
#define LINKLIST_H__

#include <stddef.h>

/* Linked link structure */
typedef struct Link_s {
    struct Link_s *next;
    struct Link_s *prev;
} Link_t;

/* Linked link list */
typedef struct {
    Link_t *head;
    Link_t *tail;
} List_t;


 /**
  Initialize linked list
  
  \param[in]  list        Linked list pointer
 */
extern void ListInit (List_t *list);

/**
  Put a link at the end of a linked list

  \param[in]  list        Linked list pointer
  \param[in]  link        Link pointer
*/
extern void ListPut (List_t *list, Link_t *link);

/**
  Put a link at the head of a linked list

  \param[in]  list        Linked list pointer
  \param[in]  link        Link pointer
*/
void ListPutHead (List_t *list, Link_t *link);

/**
  Get the first link in a linked list
  
  \param[in]  list        Linked list pointer
  \return Pointer to first link in the linked list, NULL if empty
*/
extern Link_t *ListGet (List_t *list);

/**
  Retrieve head link (no remove)
  
  \param[in]  list        Linked list pointer
  \return Pointer to head link
*/
extern Link_t *ListPeekHead (List_t *list);

/**
  Retrieve tail link (no remove)
  
  \param[in]  list        Linked list pointer
  \return Pointer to tail link
*/
extern Link_t *ListPeekTail (List_t *list);

/**
  Retrieve next link (no remove)
  
  \param[in]  link        Pointer to list link
  \return Pointer to next link
*/
extern Link_t *ListPeekNext (Link_t *link);

/**
  Retrieve previous link (no remove)
  
  \param[in]  link        Pointer to list link
  \return Pointer to next link
*/
extern Link_t *ListPeekPrev (Link_t *link);

/**
  Insert link in front of an existing link
  
  \param[in]  list        Linked list pointer
  \param[in]  link        Existing link
  \param[in]  lnew        Link to insert
*/
extern void ListInsert (List_t *list, Link_t *link, Link_t *lnew);

/**
  Remove link from the linked list
  
  \param[in]  list        Linked list pointer
  \param[in]  link        Link to remove
*/
extern void ListRemove (List_t *list, Link_t *link);

#endif /* LINKLIST_H__ */
