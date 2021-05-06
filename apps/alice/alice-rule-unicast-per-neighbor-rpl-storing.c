/**
 * Copyright (c) 2015, Swedish Institute of Computer Science.
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
 */
/**
 * \file
 *         Orchestra: a slotframe dedicated to unicast data transmission. Designed for
 *         RPL storing mode only, as this is based on the knowledge of the children (and parent).
 *         If receiver-based:
 *           Nodes listen at a timeslot defined as hash(MAC) % ORCHESTRA_SB_UNICAST_PERIOD
 *           Nodes transmit at: for each nbr in RPL children and RPL preferred parent,
 *                                             hash(nbr.MAC) % ORCHESTRA_SB_UNICAST_PERIOD
 *         If sender-based: the opposite
 *
 * \author Simon Duquennoy <simonduq@sics.se>
 */
/* for hash time measure
#include "sys/clock.h"
#include "sys/rtimer.h"
*/


#include "contiki.h"
#include "orchestra.h"
#include "net/ipv6/uip-ds6-route.h"

#include "net/ipv6/uip-ds6-nbr.h"


#include "net/packetbuf.h"
#include "net/rpl/rpl-conf.h"
#include "net/mac/tsch/tsch-private.h"//ksh..
#include "net/mac/tsch/tsch-schedule.h"//ksh..
//#include "net/mac/tsch/tsch-asn.h"
#include <stdbool.h>

#if ORCHESTRA_UNICAST_SENDER_BASED && ORCHESTRA_COLLISION_FREE_HASH
#define UNICAST_SLOT_SHARED_FLAG    ((ORCHESTRA_UNICAST_PERIOD < (ORCHESTRA_MAX_HASH + 1)) ? LINK_OPTION_SHARED : 0)
#else
#define UNICAST_SLOT_SHARED_FLAG      LINK_OPTION_SHARED
#endif

#include "net/mac/tsch/tsch-log.h"

#define DEBUG DEBUG_PRINT
#include "net/net-debug.h"


#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"


#define sq256 65536 //256*256 (square of 256)

uint16_t asfn_schedule=0; //absolute slotframe number for ALICE time varying scheduling


static uint16_t slotframe_handle = 0;
static struct tsch_slotframe *sf_unicast;

uint8_t link_option_rx = LINK_OPTION_RX ;
uint8_t link_option_tx = LINK_OPTION_TX | UNICAST_SLOT_SHARED_FLAG ; //ksh.. If it is a shared link, backoff will be applied.


uint16_t shift;
#ifdef A3_MANAGEMENT
static uint16_t CID_PERIOD; //(ORCHESTRA_UNICAST_PERIOD/A3_UNICAST_MAX_REGION)
//uint8_t cid_map[8] = {0, 4, 2, 6, 1, 5, 3, 7};


#if A3_UNICAST_MAX_REGION == 2
uint8_t cid_map[2] = {0, 1};
#elif A3_UNICAST_MAX_REGION == 4
uint8_t cid_map[4] = {0, 2, 1, 3};
#elif A3_UNICAST_MAX_REGION == 8
uint8_t cid_map[8] = {0, 4, 2, 6, 1, 5, 3, 7};
#endif



#endif


#if ALICE1_ORB2_OSB3 != 1
  linkaddr_t addrZero;
#endif


/*---------------------------------------------------------------------------*/
static uint16_t
get_node_timeslot(const linkaddr_t *addr1, const linkaddr_t *addr2, uint8_t cid) //get timeslot
{


  
#if ALICE1_ORB2_OSB3 == 2
   addr1 = &addrZero;
#elif ALICE1_ORB2_OSB3 ==3
   addr2 = &addrZero;
#endif


  if(addr1 != NULL && addr2 != NULL && ORCHESTRA_UNICAST_PERIOD > 0) { //ksh.

#ifdef A3_MANAGEMENT
 shift=(real_hash5(((uint32_t)ORCHESTRA_LINKADDR_HASH2(addr1, addr2)+(uint32_t)asfn_schedule), A3_UNICAST_MAX_REGION)+cid_map[cid])%A3_UNICAST_MAX_REGION;
 return (CID_PERIOD)*shift + real_hash5(((uint32_t)ORCHESTRA_LINKADDR_HASH2(addr1, addr2)+(uint32_t)asfn_schedule), (CID_PERIOD)); 
#else
 return real_hash5(((uint32_t)ORCHESTRA_LINKADDR_HASH2(addr1, addr2)+(uint32_t)asfn_schedule +sq256*(uint32_t)cid), (ORCHESTRA_UNICAST_PERIOD)); //link-based
#endif

  } else {
    return 0xffff;
  }
}
/*---------------------------------------------------------------------------*/
static uint16_t
get_node_channel_offset(const linkaddr_t *addr1, const linkaddr_t *addr2, uint8_t cid)
{

#if ORCHESTRA_ONE_CHANNEL_OFFSET == 1
    return slotframe_handle;
#endif


#if ALICE1_ORB2_OSB3 == 2
   addr1 = &addrZero;
#elif ALICE1_ORB2_OSB3 ==3
   addr2 = &addrZero;
#endif


 // return 2;

  int num_ch = (sizeof(TSCH_DEFAULT_HOPPING_SEQUENCE)/sizeof(uint8_t))-1; //ksh.
  if(addr1 != NULL && addr2 != NULL  && num_ch > 0) { //ksh.   
       return 1+real_hash5(((uint32_t)ORCHESTRA_LINKADDR_HASH2(addr1, addr2)+(uint32_t)asfn_schedule +sq256*(uint32_t)cid),num_ch); //link-based
  } else {
    return slotframe_handle; 
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
is_root(){
  rpl_instance_t *instance =rpl_get_default_instance();
  if(instance!=NULL && instance->current_dag!=NULL){
       uint16_t min_hoprankinc = instance->min_hoprankinc;
       uint16_t rank=(uint16_t)instance->current_dag->rank;
       if(min_hoprankinc == rank){
          return 1;
       }
  }
  return 0;
}


/*---------------------------------------------------------------------------*/
static void
alice_RESCHEDULE_unicast_slotframe(void){ //ksh.  //remove current slotframe scheduling and re-schedule this slotframe.



  uint16_t timeslot_us, timeslot_ds, channel_offset_us, channel_offset_ds;
  uint16_t timeslot_us_p, timeslot_ds_p, channel_offset_us_p, channel_offset_ds_p; //parent's schedule
  uint8_t link_option_up, link_option_down;


//remove the whole links scheduled in the unicast slotframe
  struct tsch_link *l;
  l = list_head(sf_unicast->links_list);
  if(l==NULL) return; //ksh.. 20190502  


uint8_t cid=0;



 if(is_root()!=1){ 

#ifdef A3_MANAGEMENT
for (cid=0;cid<p_num_tx_cur;cid++){
#endif
//schedule the links between parent-node and current node
     timeslot_us_p = get_node_timeslot(&linkaddr_node_addr, &orchestra_parent_linkaddr, cid);
     channel_offset_us_p = get_node_channel_offset(&linkaddr_node_addr, &orchestra_parent_linkaddr, cid);
     link_option_up=link_option_tx;

     //parent uplink schedule
     l->link_options=link_option_up;
     l->timeslot=timeslot_us_p;
     l->channel_offset=channel_offset_us_p;
     tsch_schedule_set_link_option_by_ts_choff(sf_unicast, l->timeslot, l->channel_offset, l->link_options); //ksh..20190507

     l = list_item_next(l);
     if(l==NULL) return; //ksh.. 20190502

//schedule the links between child-node and current node   //(lookup all route next hops)
#ifdef A3_MANAGEMENT
}//end for
#endif

#ifdef A3_MANAGEMENT
for (cid=0;cid<p_num_rx_cur;cid++){
#endif
     timeslot_ds_p = get_node_timeslot(&orchestra_parent_linkaddr, &linkaddr_node_addr, cid);
     channel_offset_ds_p = get_node_channel_offset(&orchestra_parent_linkaddr, &linkaddr_node_addr, cid);
     link_option_down=link_option_rx;
   
      //parent downlink schedule
      l->link_options=link_option_down;
      l->timeslot=timeslot_ds_p;
      l->channel_offset=channel_offset_ds_p;
     tsch_schedule_set_link_option_by_ts_choff(sf_unicast, l->timeslot, l->channel_offset, l->link_options); //ksh..20190507

     l = list_item_next(l);   
     if(l==NULL) return; //ksh.. 20190502  

//schedule the links between child-node and current node   //(lookup all route next hops)
#ifdef A3_MANAGEMENT
}//end for
#endif


 }//is_root()! end



  nbr_table_item_t *item = nbr_table_head(nbr_routes);
  while(l!=NULL && item!=NULL) {    

   linkaddr_t *addr = nbr_table_get_lladdr(nbr_routes, item);
#ifdef A3_MANAGEMENT
  uint8_t cid_cur_tx =1; //A3_MANAGEMENT;
  uint8_t cid_cur_rx =1; //A3_MANAGEMENT;
//  struct uip_ds6_route_neighbor_routes* it = nbr_table_get_from_lladdr(nbr_routes, addr);
  uip_ds6_nbr_t* it = nbr_table_get_from_lladdr(ds6_neighbors, addr);
  if(it != NULL){
     cid_cur_rx = it->num_rx_cur;
     cid_cur_tx = it->num_tx_cur;
  }else{
    printf("ERROR 1 \n");
  }
#endif

    

    if(linkaddr_cmp(&orchestra_parent_linkaddr, addr)){
       //item = nbr_table_next(nbr_routes, item);  //ksh..20190502

       if(item==NULL){
        
#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START //ksh sf update
           while(l!=NULL){
              //l = list_item_next(l);  //ksh.. 20190502
              //parent downlink schedule
              l->link_options=link_option_up;
              l->timeslot=timeslot_us;
              l->channel_offset=channel_offset_us;
              tsch_schedule_set_link_option_by_ts_choff(sf_unicast, l->timeslot, l->channel_offset, l->link_options); //ksh..20190507
              l = list_item_next(l);     
              if(l==NULL) return; //ksh.. 20190502

               //parent downlink schedule
               l->link_options=link_option_down;
               l->timeslot=timeslot_ds;
               l->channel_offset=channel_offset_ds;
               tsch_schedule_set_link_option_by_ts_choff(sf_unicast, l->timeslot, l->channel_offset, l->link_options); //ksh..20190507
     
	      l = list_item_next(l);  //ksh.. 20190502   
              if(l==NULL) return; //ksh.. 20190502 
           }
#endif
           return;
       }
    }


#ifdef A3_MANAGEMENT


for (cid=0;cid< cid_cur_rx; cid++){
#endif
    //ts and choff allocation

    timeslot_us = get_node_timeslot(addr, &linkaddr_node_addr, cid); 
    channel_offset_us = get_node_channel_offset(addr, &linkaddr_node_addr, cid);

    link_option_up = link_option_rx; //20190507 ksh..
/*
    //upstream link option
    if(timeslot_us==timeslot_us_p && channel_offset_us==channel_offset_us_p){
       link_option_up = link_option_tx | link_option_rx;
    }else{
       link_option_up = link_option_rx;
    }
*/
    //add links (upstream and downstream)

    //parent uplink schedule
    l->link_options=link_option_up;
    l->timeslot=timeslot_us;
    l->channel_offset=channel_offset_us;
    tsch_schedule_set_link_option_by_ts_choff(sf_unicast, l->timeslot, l->channel_offset, l->link_options); //ksh..20190507

    l = list_item_next(l); 
    if(l==NULL) return; //ksh.. 20190502    

#ifdef A3_MANAGEMENT
 } //end for (cid)
#endif




#ifdef A3_MANAGEMENT
for (cid=0;cid< cid_cur_tx;cid++){
#endif

    timeslot_ds = get_node_timeslot(&linkaddr_node_addr, addr, cid); 
    channel_offset_ds = get_node_channel_offset(&linkaddr_node_addr, addr, cid);

    link_option_down = link_option_tx; //20190507 ksh..
/*
    //downstream link option
    if(timeslot_ds==timeslot_ds_p && channel_offset_ds==channel_offset_ds_p){
       link_option_down = link_option_rx | link_option_tx;
    }else{
       link_option_down = link_option_tx;
    }
*/
     //parent downlink schedule
     l->link_options=link_option_down;
     l->timeslot=timeslot_ds;
     l->channel_offset=channel_offset_ds;
    tsch_schedule_set_link_option_by_ts_choff(sf_unicast, l->timeslot, l->channel_offset, l->link_options); //ksh..20190507
    
    l = list_item_next(l);
    if(l==NULL) return; //ksh.. 20190502

#ifdef A3_MANAGEMENT
 } //end for (cid)
#endif
    //move to the next item for while loop.
    item = nbr_table_next(nbr_routes, item);

  } //while end..




}
/*---------------------------------------------------------------------------*/
static void
alice_schedule_unicast_slotframe(void){ //ksh.  //remove current slotframe scheduling and re-schedule this slotframe.

 //printf("sch : ");

//  printf("- 4\n");
  uint16_t timeslot_us, timeslot_ds, channel_offset_us, channel_offset_ds;
  uint16_t timeslot_us_p, timeslot_ds_p, channel_offset_us_p, channel_offset_ds_p; //parent's schedule
  uint8_t link_option_up, link_option_down;

//remove the whole links scheduled in the unicast slotframe
  struct tsch_link *l;
  l = list_head(sf_unicast->links_list);
  while(l!=NULL) {    
    tsch_schedule_remove_link(sf_unicast, l);
    l = list_head(sf_unicast->links_list);
  }



uint8_t cid=0;



 if(is_root()!=1){
//schedule the links between parent-node and current node

/*
if(orchestra_parent_knows_us == 0){
printf("KSH Parent does not know us !!! \n");
}
*/

/*
    if (nbr_table_get_from_lladdr(nbr_routes, &orchestra_parent_linkaddr) !=NULL){
       printf("ERROR !! Parent in neighbor table\n");
    }
*/


   //  printf("pu ");

#ifdef A3_MANAGEMENT
for (cid=0;cid<p_num_tx_cur;cid++){
#endif

     timeslot_us_p = get_node_timeslot(&linkaddr_node_addr, &orchestra_parent_linkaddr, cid);
     channel_offset_us_p = get_node_channel_offset(&linkaddr_node_addr, &orchestra_parent_linkaddr, cid);
     link_option_up=link_option_tx;
     tsch_schedule_add_link_alice(sf_unicast, link_option_up, LINK_TYPE_NORMAL, &tsch_broadcast_address,  &orchestra_parent_linkaddr, timeslot_us_p, channel_offset_us_p);
//tsch_schedule_add_link(sf_unicast, link_option_up, LINK_TYPE_NORMAL, &tsch_broadcast_address,  timeslot_us_p, channel_offset_us_p);

  //   printf("%u ", timeslot_us_p);

#ifdef A3_MANAGEMENT
 } //end for (cid)
#endif

  //   printf("pd ");

#ifdef A3_MANAGEMENT
for (cid=0;cid<p_num_rx_cur;cid++){
#endif
     timeslot_ds_p = get_node_timeslot(&orchestra_parent_linkaddr, &linkaddr_node_addr, cid);
     channel_offset_ds_p = get_node_channel_offset(&orchestra_parent_linkaddr, &linkaddr_node_addr, cid);
     link_option_down=link_option_rx;
     tsch_schedule_add_link_alice(sf_unicast, link_option_down, LINK_TYPE_NORMAL, &tsch_broadcast_address,  &orchestra_parent_linkaddr, timeslot_ds_p, channel_offset_ds_p);
//tsch_schedule_add_link(sf_unicast, link_option_down, LINK_TYPE_NORMAL, &tsch_broadcast_address,  timeslot_ds_p, channel_offset_ds_p);

  //   printf("%u ", timeslot_ds_p);

#ifdef A3_MANAGEMENT
 } //end for (cid)
#endif

 }//is_root end

//schedule the links between child-node and current node   //(lookup all route next hops)
  nbr_table_item_t *item = nbr_table_head(nbr_routes);
  while(item != NULL) {

    linkaddr_t *addr = nbr_table_get_lladdr(nbr_routes, item);
    //ts and choff allocation
#ifdef A3_MANAGEMENT
  uint8_t cid_cur_tx = 1;// A3_MANAGEMENT;
  uint8_t cid_cur_rx = 1;// A3_MANAGEMENT;
  //struct uip_ds6_route_neighbor_routes* it = nbr_table_get_from_lladdr(nbr_routes, addr);
  uip_ds6_nbr_t* it = nbr_table_get_from_lladdr(ds6_neighbors, addr);
  if(it != NULL){
     cid_cur_rx = it->num_rx_cur;
     cid_cur_tx = it->num_tx_cur;
  }else{
    printf("ERROR 2 \n");
  }
#endif

  //   printf("u ");

#ifdef A3_MANAGEMENT
for (cid=0;cid<cid_cur_rx;cid++){
#endif

    timeslot_us = get_node_timeslot(addr, &linkaddr_node_addr, cid); 
    channel_offset_us = get_node_channel_offset(addr, &linkaddr_node_addr, cid);

  //   printf("%u ", timeslot_us);
    link_option_up = link_option_rx; //20190507 ksh..
/*
    //upstream link option
    if(timeslot_us==timeslot_us_p && channel_offset_us==channel_offset_us_p){
       link_option_up = link_option_tx | link_option_rx;
    }else{
       link_option_up = link_option_rx;
    }
*/
    //add links (upstream)
    tsch_schedule_add_link_alice(sf_unicast, link_option_up, LINK_TYPE_NORMAL, &tsch_broadcast_address, addr,  timeslot_us, channel_offset_us);
//tsch_schedule_add_link(sf_unicast, link_option_up, LINK_TYPE_NORMAL, &tsch_broadcast_address,  timeslot_us, channel_offset_us);

#ifdef A3_MANAGEMENT
 } //end for (cid)
#endif

   //  printf("d ");

#ifdef A3_MANAGEMENT
for (cid=0;cid<cid_cur_tx;cid++){
#endif

    timeslot_ds = get_node_timeslot(&linkaddr_node_addr, addr, cid); 
    channel_offset_ds = get_node_channel_offset(&linkaddr_node_addr, addr, cid);

  //   printf("%u ", timeslot_ds);
    link_option_down = link_option_tx; //20190507 ksh..
/*
    //downstream link option
    if(timeslot_ds==timeslot_ds_p && channel_offset_ds==channel_offset_ds_p){
       link_option_down = link_option_rx | link_option_tx;
    }else{
       link_option_down = link_option_tx;
    }
*/
    //add links (downstream)
    tsch_schedule_add_link_alice(sf_unicast, link_option_down, LINK_TYPE_NORMAL, &tsch_broadcast_address, addr, timeslot_ds, channel_offset_ds);
//tsch_schedule_add_link(sf_unicast, link_option_down, LINK_TYPE_NORMAL, &tsch_broadcast_address, timeslot_ds, channel_offset_ds);

#ifdef A3_MANAGEMENT
 } //end for (cid)
#endif

    //move to the next item for while loop.
    item = nbr_table_next(nbr_routes, item);
  }


//  printf("\n");

}



/*---------------------------------------------------------------------------*/ //ksh. slotframe_callback. 
#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START
void alice_callback_slotframe_start (uint16_t sfid, uint16_t sfsize){  
  asfn_schedule=sfid; //ksh.. update curr asfn_schedule.
  
// printf("sch new sf %u\n", sfid);

  //alice_RESCHEDULE_unicast_slotframe();
  alice_schedule_unicast_slotframe(); //ksh.. 20190508
}
#endif



/*---------------------------------------------------------------------------*/ //ksh. packet_selection_callback. 
#ifdef ALICE_CALLBACK_PACKET_SELECTION
int alice_callback_packet_selection (uint16_t* ts, uint16_t* choff, const linkaddr_t rx_lladdr){
//packet destination is rx_lladdr. Checks if rx_lladdr is still the node's RPL neighbor. and checks wether this can be transmitted at the current link's cell (*ts,*choff). 

  uint16_t cur_ts = *ts;
  uint16_t cur_choff = *choff;
  int is_neighbor = 0;
  uint8_t cid=0;
  //printf("ppp ");

//schedule the links between parent-node and current node
  if(linkaddr_cmp(&orchestra_parent_linkaddr, &rx_lladdr)){
     is_neighbor = 1;


#ifdef A3_MANAGEMENT
for (cid=0;cid<p_num_tx_cur;cid++){
#endif

    *ts= get_node_timeslot(&linkaddr_node_addr, &orchestra_parent_linkaddr, cid);
    *choff= get_node_channel_offset(&linkaddr_node_addr, &orchestra_parent_linkaddr, cid);       

  //  printf("p%u(%u) ", *ts, cid);
//    printf("ksh.. PCS.. parent : (%u,%u)\n", *ts, *choff);
    if(*ts == cur_ts && *choff == cur_choff){
//  printf("\n");
         return is_neighbor ; //is_neighbor =1; (parent)

    }

#ifdef A3_MANAGEMENT
 } //end for (cid)
#endif
    return is_neighbor ; //is_neighbor =1; (parent)
  }//if linkaddr_cmp (parent)







//  cid=0;

//schedule the links between child-node and current node   //(lookup all route next hops)

  nbr_table_item_t *item = nbr_table_get_from_lladdr(nbr_routes, &rx_lladdr);
  if(item != NULL){
        is_neighbor = 1;

#ifdef A3_MANAGEMENT
        uint8_t cid_cur_tx =1;// A3_MANAGEMENT;
        //  struct uip_ds6_route_neighbor_routes* it = nbr_table_get_from_lladdr(nbr_routes, addr);
        uip_ds6_nbr_t* it = nbr_table_get_from_lladdr(ds6_neighbors, &rx_lladdr);
        if(it != NULL){
          cid_cur_tx = it->num_tx_cur;
        }else{
          printf("ERROR 3 \n");
        }
#endif


#ifdef A3_MANAGEMENT
      for (cid=0;cid<cid_cur_tx;cid++){
#endif
            *ts= get_node_timeslot(&linkaddr_node_addr, &rx_lladdr, cid);
            *choff= get_node_channel_offset(&linkaddr_node_addr, &rx_lladdr, cid); 

       //   printf("%u(%u) ", *ts, cid);
            if(*ts == cur_ts && *choff == cur_choff){
       // printf("\n");
               return is_neighbor ; //is_neighbor =1; (child)
            }
#ifdef A3_MANAGEMENT
       } //end for (cid)
#endif
       return is_neighbor; //is_neighbor =1; (child)    
  }



 // this packet's receiver is not the node's RPL neighbor. 
  *ts =0;
  *choff= ALICE_BROADCAST_SF_ID;
  //-----------------------

  return is_neighbor ; // returns 0;

}
#endif //ALICE_CALLBACK_PACKET_SELECTION
/*---------------------------------------------------------------------------*/
static int
neighbor_has_uc_link(const linkaddr_t *linkaddr)
{
  if(linkaddr != NULL && !linkaddr_cmp(linkaddr, &linkaddr_null)) {
    if(orchestra_parent_knows_us 
       && linkaddr_cmp(&orchestra_parent_linkaddr, linkaddr)) {
      return 1;
    }
    if(nbr_table_get_from_lladdr(nbr_routes, (linkaddr_t *)linkaddr) != NULL ){// && !linkaddr_cmp(&orchestra_parent_linkaddr, linkaddr)) {
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
child_added(const linkaddr_t *linkaddr)
{
//printf("ksh ----------------------------- new child added\n");
#ifdef A3_MANAGEMENT
if(linkaddr != NULL){
 nbr_table_item_t *item = nbr_table_get_from_lladdr(nbr_routes, linkaddr); 
 if(item !=NULL){
   uip_ds6_nbr_t* it = nbr_table_get_from_lladdr(ds6_neighbors, linkaddr);
   if(it !=NULL){


     it->c_num_txpkt_success=0;
     it->c_num_txpkt_collision=0;
it->c_num_rxpkt_success=0;
it->c_num_rxpkt_collision=0;
it->c_num_rxpkt_idle=0;
it->c_num_rxpkt_unscheduled=0;
it->c_num_rxpkt_others=0;






     it->num_rx_cur =1; // A3_MANAGEMENT;
     it->num_tx_cur =1; // A3_MANAGEMENT;




     it->num_tx_ewma=0.4;
     it->num_rx_ewma=0.4;

   it->c_tx_collision_ewma=0.2;
   it->c_tx_attempt_ewma=0.5;
   it->c_rx_attempt_ewma=0.5;


     //printf("KSH new child added and initialized\n");
   }
 }
}
#endif



// printf("sch add child\n");
  alice_schedule_unicast_slotframe();
}
/*---------------------------------------------------------------------------*/
static void
child_removed(const linkaddr_t *linkaddr)
{
// printf("sch remove child\n");
  alice_schedule_unicast_slotframe();
}
/*---------------------------------------------------------------------------*/
static int
select_packet(uint16_t *slotframe, uint16_t *timeslot, uint16_t *channel_offset)
{
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);



  if(packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE) == FRAME802154_DATAFRAME && neighbor_has_uc_link(dest)) {
    if(slotframe != NULL) {
      *slotframe = slotframe_handle;
    }
    if(timeslot != NULL) {        
        //if the destination is the parent node, schedule it in the upstream period, if the destination is the child node, schedule it in the downstream period.
        if(linkaddr_cmp(&orchestra_parent_linkaddr, dest)){
           *timeslot = get_node_timeslot(&linkaddr_node_addr, dest, 0); //parent node (upstream)
         //  *timeslot=0;
        }else{
           *timeslot = get_node_timeslot(&linkaddr_node_addr, dest, 0);  //child node (downstream)
         //  *timeslot=0;
        }
    }
    if(channel_offset != NULL) { //ksh.
        //if the destination is the parent node, schedule it in the upstream period, if the destination is the child node, schedule it in the downstream period.
        if(linkaddr_cmp(&orchestra_parent_linkaddr, dest)){
           *channel_offset = get_node_channel_offset(&linkaddr_node_addr, dest, 0); //child node (upstream)
         //  *channel_offset=0;
        }else{
           *channel_offset = get_node_channel_offset(&linkaddr_node_addr, dest, 0); //child node (downstream)
         //  *channel_offset=0;
        }
    }

    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
new_time_source(const struct tsch_neighbor *old, const struct tsch_neighbor *new)
{
/*
uint16_t pid = get_fixed_rpl_parent_id();
if(old==NULL){
  printf("psiwtch old null\n");
}
if(new==NULL){
  printf("psiwtch new null\n");
}
if(old!=NULL && new !=NULL)
printf("pswitch %u %u -> %u %u  (%u %u)\n", old->addr.u8[LINKADDR_SIZE-2],old->addr.u8[LINKADDR_SIZE-1], new->addr.u8[LINKADDR_SIZE-2],new->addr.u8[LINKADDR_SIZE-1], pid>>8, pid^((pid>>8)<<8));
*/

//printf("ksh ----------------------------- new time source\n");
#ifdef A3_MANAGEMENT

 p_num_rx_cur =1; // A3_MANAGEMENT;
 p_num_tx_cur =1; // A3_MANAGEMENT;


 p_num_tx_ewma=0.4;
 p_num_rx_ewma=0.4;

  p_tx_collision_ewma=0.2;
  p_tx_attempt_ewma=0.5;
  p_rx_attempt_ewma=0.5;





 p_num_txpkt_success=0;
 p_num_txpkt_collision=0;

p_num_rxpkt_success=0;
p_num_rxpkt_collision=0;
p_num_rxpkt_idle=0;
p_num_rxpkt_unscheduled=0;
p_num_rxpkt_others=0;



 //printf("KSH new time source and initialized\n");
#endif




  if(new != old) {   
    const linkaddr_t *new_addr = new != NULL ? &new->addr : NULL;
    if(new_addr != NULL) {
      linkaddr_copy(&orchestra_parent_linkaddr, new_addr);    
    } else {
      linkaddr_copy(&orchestra_parent_linkaddr, &linkaddr_null);
    }
 //printf("sch new time source\n");
    alice_schedule_unicast_slotframe(); 


#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START//ksh..
            uint16_t mod=ASN_MOD(current_asn, sf_unicast->size);
            struct asn_t newasn;
            ASN_COPY(newasn, current_asn);
            ASN_DEC(newasn, mod);
            currSFID = ASN_DEVISION(newasn, sf_unicast->size);
            nextSFID = currSFID;
            //scheduledSFID = currSFID;
#endif

  }
}
/*---------------------------------------------------------------------------*/
static void
init(uint16_t sf_handle)
{
  printf("ALICE PERIOD SIZE: ORCHESTRA_UNICAST_PERIOD : %d\n",  ORCHESTRA_UNICAST_PERIOD );

  slotframe_handle = sf_handle; //sf_handle=1
  /* Slotframe for unicast transmissions */
  sf_unicast = tsch_schedule_add_slotframe(slotframe_handle, ORCHESTRA_UNICAST_PERIOD);

#if ALICE1_ORB2_OSB3 != 1
  linkaddr_copy(&addrZero,  &linkaddr_node_addr);
  addrZero.u8[LINKADDR_SIZE-2]=0;
  addrZero.u8[LINKADDR_SIZE-1]=0;
#endif



#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START
//  limitSFID = (uint16_t)((uint16_t)65535/(uint16_t)ORCHESTRA_UNICAST_PERIOD); //65535= 4Byte max value
  limitSFID = (uint16_t)((uint32_t)65536/(uint32_t)ORCHESTRA_UNICAST_PERIOD); //65535= 4Byte max value (0,65535) #65536
  printf("limitSFID: %u\n",limitSFID);
#endif 

#ifdef A3_MANAGEMENT
  CID_PERIOD = ORCHESTRA_UNICAST_PERIOD/A3_UNICAST_MAX_REGION;
  printf("A3_MANAGEMENT: %u, A3_UNICAST_MAX_REGION: %u, CID_PERIOD: %u\n", A3_MANAGEMENT, A3_UNICAST_MAX_REGION, CID_PERIOD);
#endif





#ifdef ALICE_TSCH_CALLBACK_SLOTFRAME_START
  asfn_schedule = alice_tsch_schedule_get_current_asfn(sf_unicast);//ksh..
#else
  asfn_schedule = 0; //sfid (ASN) will not be used.
#endif

}
/*---------------------------------------------------------------------------*/
struct orchestra_rule unicast_per_neighbor_rpl_storing = {
  init,
  new_time_source,
  select_packet,
  child_added,
  child_removed,
};
