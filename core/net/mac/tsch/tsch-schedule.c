/*
 * Copyright (c) 2014, SICS Swedish ICT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         IEEE 802.15.4 TSCH MAC schedule manager.
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 *         Beshr Al Nahas <beshr@sics.se>
 */

#include "contiki.h"
#include "dev/leds.h"
#include "lib/memb.h"
#include "net/nbr-table.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-queue.h"
#include "net/mac/tsch/tsch-private.h"
#include "net/mac/tsch/tsch-packet.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-log.h"
#include "net/mac/frame802154.h"
#include "sys/process.h"
#include "sys/rtimer.h"
#include <string.h>


#ifdef A3_MANAGEMENT
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#endif

/*
#if TSCH_LOG_LEVEL >= 1
#define DEBUG DEBUG_PRINT
#else // TSCH_LOG_LEVEL 
#define DEBUG DEBUG_NONE
#endif // TSCH_LOG_LEVEL 
#include "net/net-debug.h"
*/


// #define DEBUG DEBUG_NONE
 #define DEBUG DEBUG_PRINT
#include "net/net-debug.h"
/*
#define DEBUG DEBUG_NONE
#include "net/net-debug.h"
*/
//ksh. alice time varying scheduling ----------------------------//
#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START 
   void ALICE_TSCH_CALLBACK_SLOTFRAME_START(uint16_t sfid, uint16_t sfsize); 
#endif 



/* Pre-allocated space for links */
MEMB(link_memb, struct tsch_link, TSCH_SCHEDULE_MAX_LINKS);
/* Pre-allocated space for slotframes */
MEMB(slotframe_memb, struct tsch_slotframe, TSCH_SCHEDULE_MAX_SLOTFRAMES);
/* List of slotframes (each slotframe holds its own list of links) */
LIST(slotframe_list);

/* Adds and returns a slotframe (NULL if failure) */
struct tsch_slotframe *
tsch_schedule_add_slotframe(uint16_t handle, uint16_t size)
{
  if(size == 0) {
    return NULL;
  }

  if(tsch_schedule_get_slotframe_by_handle(handle)) {
    /* A slotframe with this handle already exists */
    return NULL;
  }

  if(tsch_get_lock()) {
    struct tsch_slotframe *sf = memb_alloc(&slotframe_memb);
    if(sf != NULL) {
      /* Initialize the slotframe */
      sf->handle = handle;
      ASN_DIVISOR_INIT(sf->size, size);
      LIST_STRUCT_INIT(sf, links_list);
      /* Add the slotframe to the global list */
      list_add(slotframe_list, sf);
    }
//    PRINTF("TSCH-schedule: add_slotframe %u %u\n", handle, size);
    tsch_release_lock();
    return sf;
  }   
  return NULL;
}
/*---------------------------------------------------------------------------*/
//ksh..// Thomas Wang  32bit-Interger Mix Function
uint16_t
real_hash1(uint32_t value, uint16_t mod){ //Thomas Wang method.. and  Robert Jenkins' method.. sequentially

  uint32_t a=value;
 
  a = (a ^ 61) ^ (a >> 16);
  a = a + (a << 3);
  a = a ^ (a >> 4);
  a = a * 0x27d4eb2d;
  a = a ^ (a >> 15);


   a = (a+0x7ed55d16) + (a<<12);
   a = (a^0xc761c23c) ^ (a>>19);
   a = (a+0x165667b1) + (a<<5);
   a = (a+0xd3a2646c) ^ (a<<9);
   a = (a+0xfd7046c5) + (a<<3);
   a = (a^0xb55a4f09) ^ (a>>16);
  
  return (uint16_t)(a% (uint32_t)mod);
}


/*---------------------------------------------------------------------------*/
//ksh..// Thomas Wang  32bit-Interger Mix Function
uint16_t
real_hash(uint32_t value, uint16_t mod){ //Thomas Wang method..

  uint32_t a=value;
 
  a = (a ^ 61) ^ (a >> 16);
  a = a + (a << 3);
  a = a ^ (a >> 4);
  a = a * 0x27d4eb2d;
  a = a ^ (a >> 15);
  
  return (uint16_t)(a% (uint32_t)mod);
}
/*---------------------------------------------------------------------------*/
//ksh..// Thomas Wang  32bit-Interger Mix Function
uint16_t
real_hash5(uint32_t value, uint16_t mod){ //ksh..

  uint32_t a=value;
  a = ((((a + (a>>16)) ^ (a>>9)) ^ (a<<3)) ^ (a>>5));
  return (uint16_t)(a% (uint32_t)mod);
}
/*---------------------------------------------------------------------------*/
//ksh.. //  //https : gist.github.com/badboy/6267743  Robert Jenkins' 32 bit integer hash function
uint16_t
real_hash3(uint32_t value, uint16_t mod){ //

  uint32_t a=value;

   a = (a+0x7ed55d16) + (a<<12);
   a = (a^0xc761c23c) ^ (a>>19);
   a = (a+0x165667b1) + (a<<5);
   a = (a+0xd3a2646c) ^ (a<<9);
   a = (a+0xfd7046c5) + (a<<3);
   a = (a^0xb55a4f09) ^ (a>>16);

  return (uint16_t)(a% (uint32_t)mod);
}

/*---------------------------------------------------------------------------*/
//ksh.. //  //https : gist.github.com/badboy/6267743  Multiplication method
uint16_t
real_hash2(uint32_t value, uint16_t mod){ //

  uint32_t key=value;
  uint32_t c2=0x27d4eb2d; // a prime or an odd constant
  key = (key ^ 61) ^ (key >> 16);
  key = key + (key << 3);
  key = key ^ (key >> 4);
  key = key * c2;
  key = key ^ (key >> 15);

  return (uint16_t)(key% (uint32_t)mod);
}

/*---------------------------------------------------------------------------*/
//ksh. remove link by timeslot and channel offset
int
tsch_schedule_remove_link_by_ts_choff(struct tsch_slotframe *slotframe, uint16_t timeslot, uint16_t channel_offset)
{
  return slotframe != NULL &&
         tsch_schedule_remove_link(slotframe, tsch_schedule_get_link_by_ts_choff(slotframe, timeslot, channel_offset));
}/*---------------------------------------------------------------------------*/
//ksh. get link by timeslot and channel offset
struct tsch_link *
tsch_schedule_get_link_by_ts_choff(struct tsch_slotframe *slotframe, uint16_t timeslot, uint16_t channel_offset)
{
 // if(!tsch_is_locked()) {
    if(slotframe != NULL) {
      struct tsch_link *l = list_head(slotframe->links_list);
      /* Loop over all items. Assume there is max one link per timeslot */
      while(l != NULL) {
        if(l->timeslot == timeslot && l->channel_offset == channel_offset) {
          return l;
        }
        l = list_item_next(l);
      }
      return l;
    }
 // }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Removes all slotframes, resulting in an empty schedule */
int
tsch_schedule_remove_all_slotframes(void)
{
  struct tsch_slotframe *sf;
  while((sf = list_head(slotframe_list))) {
    if(tsch_schedule_remove_slotframe(sf) == 0) {
      return 0;
    }
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/* Removes a slotframe Return 1 if success, 0 if failure */
int
tsch_schedule_remove_slotframe(struct tsch_slotframe *slotframe)
{
  if(slotframe != NULL) {
    /* Remove all links belonging to this slotframe */
    struct tsch_link *l;
    while((l = list_head(slotframe->links_list))) {
      tsch_schedule_remove_link(slotframe, l);
    }

    /* Now that the slotframe has no links, remove it. */
    if(tsch_get_lock()) {
//      PRINTF("TSCH-schedule: remove slotframe %u %u\n", slotframe->handle, slotframe->size.val);
      memb_free(&slotframe_memb, slotframe);
      list_remove(slotframe_list, slotframe);
      tsch_release_lock();
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Looks for a slotframe from a handle */
struct tsch_slotframe *
tsch_schedule_get_slotframe_by_handle(uint16_t handle)
{
 // if(!tsch_is_locked()) {  //ksh..
    struct tsch_slotframe *sf = list_head(slotframe_list);
    while(sf != NULL) {
      if(sf->handle == handle) {
        return sf;
      }
      sf = list_item_next(sf);
    }
 // }
  return NULL;
}
/*---------------------------------------------------------------------------*/
/* Looks for a link from a handle */
struct tsch_link *
tsch_schedule_get_link_by_handle(uint16_t handle)
{
  //if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);
    while(sf != NULL) {
      struct tsch_link *l = list_head(sf->links_list);
      /* Loop over all items. Assume there is max one link per timeslot */
      while(l != NULL) {
        if(l->handle == handle) {
          return l;
        }
        l = list_item_next(l);
      }
      sf = list_item_next(sf);
    }
  //}
  return NULL;
}/*---------------------------------------------------------------------------*/
#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START //ksh sf update
//ksh. set link option by timeslot and channel offset
struct tsch_link *
tsch_schedule_set_link_option_by_ts_choff(struct tsch_slotframe *slotframe, uint16_t timeslot, uint16_t channel_offset, uint8_t* link_options) //ksh ..
{

 // if(!tsch_is_locked()) { //ksh..
    if(slotframe != NULL) {
      struct tsch_link *l = list_head(slotframe->links_list);
      /* Loop over all items. Assume there is max one link per timeslot */
      while(l != NULL) {
        if(l->timeslot == timeslot && l->channel_offset == channel_offset) {
          *link_options |= l->link_options;//ksh..
          l->link_options |= *link_options;
        }
        l = list_item_next(l);
      }
      return l;
    }
 // }
  return NULL;
}
#endif
/*---------------------------------------------------------------------------*/
/* Adds a link to a slotframe, return a pointer to it (NULL if failure) */
struct tsch_link *
tsch_schedule_add_link(struct tsch_slotframe *slotframe,
                       uint8_t link_options, enum link_type link_type, const linkaddr_t *address,
                       uint16_t timeslot, uint16_t channel_offset)
{
  struct tsch_link *l = NULL;
  if(slotframe != NULL) {
    /* We currently support only one link per timeslot in a given slotframe. */
    /* Start with removing the link currently installed at this timeslot (needed
     * to keep neighbor state in sync with link options etc.) */
//    tsch_schedule_remove_link_by_timeslot(slotframe, timeslot);

    tsch_schedule_remove_link_by_ts_choff(slotframe, timeslot, channel_offset);

//    if(!tsch_get_lock()) {
    if(0) { //ksh.. 
//      PRINTF("TSCH-schedule:! add_link memb_alloc couldn't take lock\n");
    } else {
      l = memb_alloc(&link_memb);
      if(l == NULL) {
  //      PRINTF("TSCH-schedule:! add_link memb_alloc failed\n");
        tsch_release_lock();
      } else {
        static int current_link_handle = 0;
        struct tsch_neighbor *n;
        /* Add the link to the slotframe */
        list_add(slotframe->links_list, l);
        /* Initialize link */
        l->handle = current_link_handle++;
        l->link_options = link_options;
        l->link_type = link_type;
        l->slotframe_handle = slotframe->handle;
        l->timeslot = timeslot;
        l->channel_offset = channel_offset;
        l->data = NULL;
        if(address == NULL) {
          address = &linkaddr_null;
        }
        linkaddr_copy(&l->addr, address);

        /* Release the lock before we update the neighbor (will take the lock) */
        tsch_release_lock();

        if(l->link_options & LINK_OPTION_TX) {
          n = tsch_queue_add_nbr(&l->addr);
          /* We have a tx link to this neighbor, update counters */
          if(n != NULL) {
            n->tx_links_count++;
            if(!(l->link_options & LINK_OPTION_SHARED)) {
              n->dedicated_tx_links_count++;
            }
          }
        }
      }
    }
  }
  return l;
}
/*---------------------------------------------------------------------------*/
/* Adds a link to a slotframe, return a pointer to it (NULL if failure) */
struct tsch_link *
tsch_schedule_add_link_alice(struct tsch_slotframe *slotframe,
                       uint8_t link_options, enum link_type link_type, const linkaddr_t *address, const linkaddr_t *neighbor,
                       uint16_t timeslot, uint16_t channel_offset)
{
  struct tsch_link *l = NULL;
  if(slotframe != NULL) {

//    if(!tsch_get_lock()) {
    if(0) { //ksh.. 
//      PRINTF("TSCH-schedule:! add_link memb_alloc couldn't take lock\n");
    } else {
      l = memb_alloc(&link_memb);
      if(l == NULL) {
  //      PRINTF("TSCH-schedule:! add_link memb_alloc failed\n");
        tsch_release_lock();
      } else {
        static int current_link_handle = 0;
        struct tsch_neighbor *n;
        /* Add the link to the slotframe */
        list_add(slotframe->links_list, l);
        /* Initialize link */
        l->handle = current_link_handle++;

#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START //ksh sf update
        l->link_option_alice = link_options;//ksh.. //neighbor's original setting.
        tsch_schedule_set_link_option_by_ts_choff(slotframe, timeslot, channel_offset, &link_options); //link_option is updated after the function.
#endif
        l->link_options = link_options;
        l->link_type = link_type;
        l->slotframe_handle = slotframe->handle;
        l->timeslot = timeslot;
        l->channel_offset = channel_offset;
        l->data = NULL;
        if(address == NULL) {
          address = &linkaddr_null;
        }
        linkaddr_copy(&l->addr, address);
        if(neighbor == NULL) {
          neighbor = &linkaddr_null;
        }
        linkaddr_copy(&l->neighbor, neighbor);

        /* Release the lock before we update the neighbor (will take the lock) */
        tsch_release_lock();

        if(l->link_options & LINK_OPTION_TX) {
          n = tsch_queue_add_nbr(&l->addr);
          /* We have a tx link to this neighbor, update counters */
          if(n != NULL) {
            n->tx_links_count++;
            if(!(l->link_options & LINK_OPTION_SHARED)) {
              n->dedicated_tx_links_count++;
            }
          }
        }

      }//else end
    }
  }
  return l;
}
/*---------------------------------------------------------------------------*/
/* Removes a link from slotframe. Return 1 if success, 0 if failure */
int
tsch_schedule_remove_link(struct tsch_slotframe *slotframe, struct tsch_link *l)
{
  if(slotframe != NULL && l != NULL && l->slotframe_handle == slotframe->handle) {
//    if(tsch_get_lock()) {
    if(1) { //ksh.. 
      uint8_t link_options;
      linkaddr_t addr;

      /* Save link option and addr in local variables as we need them
       * after freeing the link */
      link_options = l->link_options;
      linkaddr_copy(&addr, &l->addr);

      /* The link to be removed is scheduled as next, set it to NULL
       * to abort the next link operation */
      if(l == current_link) {
        current_link = NULL;
      }
      /* PRINTF("TSCH-schedule: remove_link %u %u %u %u %u\n",
             slotframe->handle, l->link_options, l->timeslot, l->channel_offset,
             TSCH_LOG_ID_FROM_LINKADDR(&l->addr));
*/
      list_remove(slotframe->links_list, l);
      memb_free(&link_memb, l);

      /* Release the lock before we update the neighbor (will take the lock) */
      tsch_release_lock();

      /* This was a tx link to this neighbor, update counters */
      if(link_options & LINK_OPTION_TX) {
        struct tsch_neighbor *n = tsch_queue_add_nbr(&addr);
        if(n != NULL) {
          n->tx_links_count--;
          if(!(link_options & LINK_OPTION_SHARED)) {
            n->dedicated_tx_links_count--;
          }
        }
      }

      return 1;
    } else {
     // PRINTF("TSCH-schedule:! remove_link memb_alloc couldn't take lock\n");
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Removes a link from slotframe and timeslot. Return a 1 if success, 0 if failure */
int
tsch_schedule_remove_link_by_timeslot(struct tsch_slotframe *slotframe, uint16_t timeslot)
{
  return slotframe != NULL &&
         tsch_schedule_remove_link(slotframe, tsch_schedule_get_link_by_timeslot(slotframe, timeslot));
}
/*---------------------------------------------------------------------------*/
/* Looks within a slotframe for a link with a given timeslot */
struct tsch_link *
tsch_schedule_get_link_by_timeslot(struct tsch_slotframe *slotframe, uint16_t timeslot)
{
  if(!tsch_is_locked()) {
    if(slotframe != NULL) {
      struct tsch_link *l = list_head(slotframe->links_list);
      /* Loop over all items. Assume there is max one link per timeslot */
      while(l != NULL) {
        if(l->timeslot == timeslot) {
          return l;
        }
        l = list_item_next(l);
      }
      return l;
    }
  }
  return NULL;
}

//.......................................................
/*---------------------------------------------------------------------------*/
//ksh. alice time varying schedule
#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START
void tsch_schedule_alice_data_Sf_reschedule(struct asn_t *asn){

  int flag=0;

   struct tsch_slotframe *sf = tsch_schedule_get_slotframe_by_handle(ALICE_UNICAST_SF_ID);
   if(sf==NULL){ 
      return; 
   }

   if( (currSFID==0 && nextSFID==limitSFID) || (  !(currSFID==0 && nextSFID==limitSFID) && currSFID > nextSFID)  ){
      nextSFID=currSFID;
      ALICE_TSCH_CALLBACK_SLOTFRAME_START(nextSFID, (uint16_t)sf->size.val);
   }

   uint16_t timeslot = ASN_MOD(*asn, sf->size);
   struct tsch_link *l = list_head(sf->links_list);
   while(l != NULL) {        
     if(l->timeslot > timeslot){ //ksh. Check if any link exists after this timeslot in the current unicast slotframe
            flag=1; //In the current unicast slotframe schedule, the current timeslot is not a last one.
     }
     l = list_item_next(l);
   }//ksh.. while end .. link


  if(flag==0 && currSFID == nextSFID){//It means that the current timeslot is the last timeslot scheduled on the current slotframe. update ASFN and reschedule the slotframe.
    if(currSFID==limitSFID ){
       nextSFID = 0;
    }else{
       nextSFID = currSFID+1;
    }

    rtimer_clock_t rt6= RTIMER_NOW();

#ifdef A3_MANAGEMENT
    //STATIC variables: initialized only once.
    double alpha= 0.02 + ((double)ORCHESTRA_CONF_UNICAST_PERIOD*(double)0.001); // 20-80 -> 0.04 - 0.1 // max: 0.15
    if(alpha > 0.15){ alpha = 0.15;}
    double alphaPlus= alpha*0.2;
    double dynamicAlpha=0;


    static double txIncreaseThresh = 0.75;
    static double txDecreaseThresh = 0.36; //0.34; //0.375
    static double rxIncreaseThresh = 0.65;
    static double rxDecreaseThresh = 0.29; //0.29; //325; 

    static double maxEr = 0.5; // collision prob

   uint8_t sumtx=0;
   uint8_t sumrx=0;
   uint8_t diffrx=0;
   double newval = 0;



//   rpl_instance_t *instance =rpl_get_default_instance();
//   if(instance!=NULL && instance->current_dag!=NULL && instance->current_dag->preferred_parent!=NULL){


//-------------------------------------------------------------------
     sumtx = p_num_txpkt_success + p_num_txpkt_collision;
     newval = (double)(sumtx)/(double)(p_num_tx_cur);
     if(newval > 1) { newval = 1; }
     p_tx_attempt_ewma = (1-alpha)*p_tx_attempt_ewma + alpha*newval;

     newval = (double)(p_num_txpkt_success)/(double)(p_num_tx_cur);
     if(newval > 1) { newval = 1; }
     p_num_tx_ewma = (1-alpha)*p_num_tx_ewma + alpha*newval;
//-------------------------------------------------------------------

     sumrx = p_num_rxpkt_collision + p_num_rxpkt_success + p_num_rxpkt_idle + p_num_rxpkt_others;
     diffrx=(p_num_rx_cur - sumrx); //difference
     if(p_num_rx_cur < sumrx){ //minus value
       diffrx=0;
     }
     p_num_rxpkt_unscheduled = diffrx;
     sumrx += diffrx;

     newval = ((double)p_num_rxpkt_success + p_rx_attempt_ewma*(double)(p_num_rxpkt_collision+p_num_rxpkt_unscheduled))/(double)(sumrx);
     if(newval > 1) { newval = 1; }
     dynamicAlpha=alpha;
     if(newval > p_rx_attempt_ewma){dynamicAlpha += alphaPlus;}
     p_rx_attempt_ewma=(1-dynamicAlpha)*p_rx_attempt_ewma + dynamicAlpha*newval;

//-------------------------------------------------------------------

     int txChangeFlag=0;



#if ALICE1_ORB2_OSB3 != 2 //O-SB, ALICE
//     if(p_tx_collision_ewma > maxEr && p_num_tx_cur >1){
     if(( p_tx_attempt_ewma -p_num_tx_ewma)/(p_tx_attempt_ewma) > maxEr && p_num_tx_cur >1){

  	p_num_tx_cur = p_num_tx_cur/2;
	txChangeFlag = 1;

	p_tx_attempt_ewma =0.5;
        p_tx_collision_ewma = 0.2;	
	p_num_tx_ewma =0.4;

     }
#endif


     if(txChangeFlag == 0){
	if(p_tx_attempt_ewma > txIncreaseThresh && p_num_tx_cur < A3_UNICAST_MAX_REGION){
		p_num_tx_cur = p_num_tx_cur*2;
		p_tx_attempt_ewma /=2;
	}else if(p_tx_attempt_ewma < txDecreaseThresh && p_num_tx_cur > 1){
		p_num_tx_cur = p_num_tx_cur/2;
		p_tx_attempt_ewma *=2;
	}
     } //txFlagChangedForChild == false


     if(p_rx_attempt_ewma > rxIncreaseThresh && p_num_rx_cur < A3_UNICAST_MAX_REGION){
	p_num_rx_cur = p_num_rx_cur*2;
	p_rx_attempt_ewma /=2;
     }else if(p_rx_attempt_ewma < rxDecreaseThresh && p_num_rx_cur > 1){
	p_num_rx_cur = p_num_rx_cur/2;
	p_rx_attempt_ewma *=2;
     }

//   } // has rpl parent

   p_num_txpkt_success=0;
   p_num_txpkt_collision=0;

   p_num_rxpkt_success=0;
   p_num_rxpkt_collision=0;
   p_num_rxpkt_idle=0;
   p_num_rxpkt_unscheduled=0;
   p_num_rxpkt_others=0;


   nbr_table_item_t *item = nbr_table_head(nbr_routes);
   while( item!=NULL) {    
     linkaddr_t *addr = nbr_table_get_lladdr(nbr_routes, item);
     if(addr != NULL){
       uip_ds6_nbr_t* it = nbr_table_get_from_lladdr(ds6_neighbors, addr);
       if(it !=NULL){       


//-------------------------------------------------------------------
     sumtx = it->c_num_txpkt_success + it->c_num_txpkt_collision;
     newval = (double)(sumtx)/(double)(it->num_tx_cur);
     if(newval > 1) { newval = 1; }
     it->c_tx_attempt_ewma = (1-alpha)*it->c_tx_attempt_ewma + alpha*newval;

     newval = (double)(it->c_num_txpkt_success)/(double)(it->num_tx_cur);
     if(newval > 1) { newval = 1; }
     it->num_tx_ewma = (1-alpha)*it->num_tx_ewma + alpha*newval;
//-------------------------------------------------------------------
     sumrx = it->c_num_rxpkt_collision + it->c_num_rxpkt_success + it->c_num_rxpkt_idle + it->c_num_rxpkt_others;
     diffrx=(it->num_rx_cur - sumrx); // differance
     if(it->num_rx_cur < sumrx){
       diffrx=0;
     }
     it->c_num_rxpkt_unscheduled = diffrx;
     sumrx += diffrx;

     newval = ((double)it->c_num_rxpkt_success + it->c_rx_attempt_ewma*(double)(it->c_num_rxpkt_collision + it->c_num_rxpkt_unscheduled))/(double)(sumrx);
     if(newval > 1) { newval = 1; }
     dynamicAlpha=alpha;
     if(newval > it->c_rx_attempt_ewma){dynamicAlpha += alphaPlus;}
     it->c_rx_attempt_ewma=(1-dynamicAlpha)*it->c_rx_attempt_ewma + dynamicAlpha*newval;

//-------------------------------------------------------------------

	 int txFlagChangedForChild = 0;

#if ALICE1_ORB2_OSB3 != 2 //O-SB, ALICE
//         if(it->c_tx_collision_ewma > maxEr && it->num_tx_cur > 1){
         if((it->c_tx_attempt_ewma - it->num_tx_ewma)/(it->c_tx_attempt_ewma) > maxEr && it->num_tx_cur > 1){
           
	   it->num_tx_cur = it->num_tx_cur/2;
	   txFlagChangedForChild = 1;

	   it->c_tx_collision_ewma =0.2;
	   it->c_tx_attempt_ewma =0.5;
	   it->num_tx_ewma =0.4;

         }
#endif

	if(txFlagChangedForChild == 0){
		if(it->c_tx_attempt_ewma > txIncreaseThresh && it->num_tx_cur < A3_UNICAST_MAX_REGION){
			it->num_tx_cur = it->num_tx_cur*2;
			it->c_tx_attempt_ewma /=2;
		}else if(it->c_tx_attempt_ewma < txDecreaseThresh && it->num_tx_cur > 1){
			it->num_tx_cur = it->num_tx_cur/2;
			it->c_tx_attempt_ewma *=2;
		}
	} //txFlagChangedForChild == false



	if(it->c_rx_attempt_ewma > rxIncreaseThresh && it->num_rx_cur < A3_UNICAST_MAX_REGION){
                it->num_rx_cur = it->num_rx_cur*2;
                it->c_rx_attempt_ewma /=2;
        }else if(it->c_rx_attempt_ewma < rxDecreaseThresh && it->num_rx_cur > 1){
                it->num_rx_cur = it->num_rx_cur/2;
                it->c_rx_attempt_ewma *=2;
        }


         it->c_num_rxpkt_success=0;
         it->c_num_rxpkt_collision=0;
         it->c_num_rxpkt_idle=0;
         it->c_num_rxpkt_unscheduled=0;
         it->c_num_rxpkt_others=0;

         it->c_num_txpkt_success=0;
         it->c_num_txpkt_collision=0;

       }// if it!=NULL
     }

     item = nbr_table_next(nbr_routes, item);
   } //end while

#endif //A3_MANAGEMENT


   rtimer_clock_t rt7= RTIMER_NOW();
   ALICE_TSCH_CALLBACK_SLOTFRAME_START(nextSFID, (uint16_t)sf->size.val);
   rtimer_clock_t rt8= RTIMER_NOW();


 }//if end //if(flag==0 && currSFID == nextSFID){

//}//ksh..
}


#endif//ksh.
/*---------------------------------------------------------------------------*/
/* Returns the next active link after a given ASN, and a backup link (for the same ASN, with Rx flag) */
struct tsch_link *
tsch_schedule_get_next_active_link(struct asn_t *asn, uint16_t *time_offset,
    struct tsch_link **backup_link)
{
	//printf("GNE\n");
  uint16_t time_to_curr_best = 0;
  struct tsch_link *curr_best = NULL;
  struct tsch_link *curr_backup = NULL; /* Keep a back link in case the current link
  turns out useless when the time comes. For instance, for a Tx-only link, if there is
  no outgoing packet in queue. In that case, run the backup link instead. The backup link
  must have Rx flag set. */
//   if(!tsch_is_locked()) {

    struct tsch_slotframe *sf = list_head(slotframe_list);
    /* For each slotframe, look for the earliest occurring link */
    while(sf != NULL) {

#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START//ksh..
    if(sf->handle == ALICE_UNICAST_SF_ID ){

            uint16_t mod=ASN_MOD(*asn, sf->size);
            struct asn_t newasn;
            ASN_COPY(newasn, *asn);
            ASN_DEC(newasn, mod);
            currSFID = ASN_DEVISION(newasn, sf->size);
            tsch_schedule_alice_data_Sf_reschedule(asn); //ALICE time varying scheduling
    }
#endif


      /* Get timeslot from ASN, given the slotframe length */
      uint16_t timeslot = ASN_MOD(*asn, sf->size);
      struct tsch_link *l = list_head(sf->links_list);
      while(l != NULL) {

        uint16_t time_to_timeslot=
          l->timeslot > timeslot ?
          l->timeslot - timeslot :
          sf->size.val + l->timeslot - timeslot; 

#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START//ksh..
    if(sf->handle == ALICE_UNICAST_SF_ID && currSFID !=nextSFID && l->timeslot > timeslot){
       time_to_timeslot = 999; //maximum value;
    }
#endif

        if(curr_best == NULL || time_to_timeslot < time_to_curr_best) { //ksh.. initialize curr_best.
          time_to_curr_best = time_to_timeslot;
          curr_best = l;
          curr_backup = NULL;
        } else if(time_to_timeslot == time_to_curr_best && l->channel_offset != curr_best->channel_offset) { //ksh.. It is not the same link
          struct tsch_link *new_best = NULL;
          /* Two links are overlapping, we need to select one of them.
           * By standard: prioritize Tx links first, second by lowest handle */
          if((curr_best->link_options & LINK_OPTION_TX) == (l->link_options & LINK_OPTION_TX)) {
            /* Both or neither links have Tx, select the one with lowest handle */
            if(l->slotframe_handle < curr_best->slotframe_handle) {
              new_best = l;
            }

#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START
            //ksh..
            /* Both or neither links have Tx, and has the same priority. select the one with a longer queue */
            else if(l->slotframe_handle == ALICE_UNICAST_SF_ID && curr_best->slotframe_handle == ALICE_UNICAST_SF_ID){
               if(tsch_queue_packet_count(&(l->neighbor)) > tsch_queue_packet_count(&(curr_best->neighbor))){
                  new_best = l;
               }
            }
#endif
          } else { 
            /* Select the link that has the Tx option */
            if(l->link_options & LINK_OPTION_TX) {
              new_best = l;
            }
          }

          /* Maintain backup_link */
          if(curr_backup == NULL) {
            /* Check if 'l' best can be used as backup */
            if(new_best != l && (l->link_options & LINK_OPTION_RX)) { /* Does 'l' have Rx flag? */
              curr_backup = l;
            }
            /* Check if curr_best can be used as backup */
            if(new_best != curr_best && (curr_best->link_options & LINK_OPTION_RX)) { /* Does curr_best have Rx flag? */
              curr_backup = curr_best;
            }
          }

          /* Maintain curr_best */
          if(new_best != NULL) {
            curr_best = new_best;
          }
        }
        l = list_item_next(l);
      }//ksh.. while end .. link
      sf = list_item_next(sf);
    }//ksh.. while end .. sf
    if(time_offset != NULL) {
      *time_offset = time_to_curr_best;
    }
//  } // if(!tsch_is_locked()) {


  if(backup_link != NULL) {
    *backup_link = curr_backup;
  }

  return curr_best;
}
/*---------------------------------------------------------------------------*/
/* Module initialization, call only once at startup. Returns 1 is success, 0 if failure. */
int
tsch_schedule_init(void)
{
  if(tsch_get_lock()) {
    memb_init(&link_memb);
    memb_init(&slotframe_memb);
    list_init(slotframe_list);
    tsch_release_lock();
    return 1;
  } else {
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
/* Create a 6TiSCH minimal schedule */
void
tsch_schedule_create_minimal(void)
{
  struct tsch_slotframe *sf_min;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* Build 6TiSCH minimal schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_min = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);
  /* Add a single Tx|Rx|Shared slot using broadcast address (i.e. usable for unicast and broadcast).
   * We set the link type to advertising, which is not compliant with 6TiSCH minimal schedule
   * but is required according to 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_min,
      LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED | LINK_OPTION_TIME_KEEPING,
      LINK_TYPE_ADVERTISING, &tsch_broadcast_address,
      0, 0);
}
/*---------------------------------------------------------------------------*/
/* Prints out the current schedule (all slotframes and links) */
void
tsch_schedule_print(void)
{
  if(!tsch_is_locked()) {
    struct tsch_slotframe *sf = list_head(slotframe_list);

//    printf("Schedule: slotframe list\n");

    while(sf != NULL) {
      struct tsch_link *l = list_head(sf->links_list);

  //    printf("[Slotframe] Handle %u, size %u\n", sf->handle, sf->size.val);
    //  printf("List of links:\n");

      while(l != NULL) {
      /*  printf("[Link] Options %02x, type %u, timeslot %u, channel offset %u, address %u\n",
               l->link_options, l->link_type, l->timeslot, l->channel_offset, l->addr.u8[7]);
*/
        l = list_item_next(l);
      }

      sf = list_item_next(sf);
    }

//    printf("Schedule: end of slotframe list\n");
  }
}
/*---------------------------------------------------------------------------*/
