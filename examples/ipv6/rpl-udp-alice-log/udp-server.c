/*
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

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"


#if WITH_COMPOWER & SERVER_WITH_COMPOWER
#include "powertrace.h"
#endif

#include "node-id.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/mac/tsch/tsch.h"
#include "net/rpl/rpl-private.h"
//#include "dev/temperature-sensor.h"
//#include "board.h"

#if WITH_ORCHESTRA
#include "orchestra.h"
#endif /* WITH_ORCHESTRA */



#define CONFIG_VIA_BUTTON PLATFORM_HAS_BUTTON

/*---------------------------------------------------------------------------*/
/*PROCESS(node_process, "RPL Node");
#if CONFIG_VIA_BUTTON
AUTOSTART_PROCESSES(&node_process, &sensors_process);
#else 
AUTOSTART_PROCESSES(&node_process);
#endif 
*/

#if !ECHO_DOWNSTREAM_ENABLED //ksh..
#include "lib/list.h"
#endif




#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define UDP_EXAMPLE_ID  190


#define MAX_PAYLOAD_LEN		20

static struct uip_udp_conn *server_conn;
static int seq_id=1;




PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);


#if !ECHO_DOWNSTREAM_ENABLED //ksh..
uip_ipaddr_t route_copy_list[MAX_NODE_NUM];
int num_child=0;

#endif



void
print_mac_states(){

#if WITH_COMPOWER & SERVER_WITH_COMPOWER


printf("m mactx: %d %d %d %d %d %d %d %u %u %u %u %u\n", mac_tx_up_ok_counter,  mac_tx_up_error_counter, mac_tx_down_ok_counter, mac_tx_down_error_counter, dc_radio, dc_tx, dc_listen,num_pktdrop_queue, num_pktdrop_mac, num_pktdrop_rpl, num_dis+num_dio+num_dao+num_dao_ack, tsch_queue_overflow);





#else
    PRINTF("m mactx: %d %d %d %d %d %d %d %d %d %d %d %d\n", mac_tx_up_ok_counter, mac_tx_up_collision_counter, mac_tx_up_noack_counter, mac_tx_up_deferred_counter, mac_tx_up_err_counter, mac_tx_up_err_fatal_counter,     mac_tx_down_ok_counter, mac_tx_down_collision_counter, mac_tx_down_noack_counter, mac_tx_down_deferred_counter, mac_tx_down_err_counter, mac_tx_down_err_fatal_counter);
#endif
}

void
send_packet(uip_ipaddr_t *ipaddr){

   if(seq_id > MAX_NUM_DOWNSTREAM_PACKETS) { 
      return; 
   }
    //printf("ENERGEST_CONF_ON: %d\n", ENERGEST_CONF_ON);
    PRINTF("D msg %d to %u %u t %u\n", seq_id++, ipaddr->u8[sizeof(ipaddr->u8)-2], ipaddr->u8[sizeof(ipaddr->u8)-1], (uint16_t) current_asn.ls4b);  //modified

    char buf[MAX_PAYLOAD_LEN];
    sprintf(buf, "%u", (uint16_t) current_asn.ls4b);

    uip_ipaddr_copy(&server_conn->ripaddr, ipaddr);// &UIP_IP_BUF->srcipaddr);
    uip_udp_packet_send(server_conn, buf, strlen(buf));
    uip_create_unspecified(&server_conn->ripaddr);




#if !ECHO_DOWNSTREAM_ENABLED
#if DO_SEQ_SEND_WITH_INTERVAL
    num_child-=1;
#endif
#endif

}


int address_flag=0;

#if !ECHO_DOWNSTREAM_ENABLED

static void
send_process()
{




#ifdef A3_MANAGEMENT
#if LQ_PRINT == 1
//----------------------------------------------------------------------
        printf("\ncollect info %u\n\n", currSFID);
       
        nbr_table_item_t *item = nbr_table_head(nbr_routes);
        while( item!=NULL) {    
          linkaddr_t *addr = nbr_table_get_lladdr(nbr_routes, item);
          if(addr != NULL){
            uip_ds6_nbr_t* it = nbr_table_get_from_lladdr(ds6_neighbors, addr);
            if(it !=NULL){       

                 printf("col %u %u %u %u %u %u %u rx %.2f %u tx %.2f %u\n", linkaddr_node_addr.u8[LINKADDR_SIZE-2], linkaddr_node_addr.u8[LINKADDR_SIZE-1], addr->u8[LINKADDR_SIZE-2], addr->u8[LINKADDR_SIZE-1], (uint16_t)(100*it->c_tx_attempt_ewma), 0, (uint16_t)(100*it->c_rx_attempt_ewma), it->num_rx_ewma, it->num_rx_cur,  it->num_tx_ewma, it->num_tx_cur);

            }
          }
          item = nbr_table_next(nbr_routes, item);
        }
//----------------------------------------------------------------------
#endif
#endif





    PRINTF("-------------SEND PROCESS START ! (MAKE DOWNSTREAM DESTINATION LIST)\n");



 num_child=61;
 if(address_flag ==0){
    address_flag = 1;

    uip_ds6_route_t *route;
    route = uip_ds6_route_head();

    int i;
    for(i=0;i<num_child;i++){
       uip_ipaddr_copy(&route_copy_list[i], &route->ipaddr);
    }


//grenoble 2
route_copy_list[ 0 ].u8[14]= 160 ;   route_copy_list[ 0 ].u8[15]= 120 ;  // 290
route_copy_list[ 1 ].u8[14]= 145 ;   route_copy_list[ 1 ].u8[15]= 131 ;  // 292
route_copy_list[ 2 ].u8[14]= 151 ;   route_copy_list[ 2 ].u8[15]= 129 ;  // 351
//route_copy_list[ 2 ].u8[14]= 148 ;   route_copy_list[ 2 ].u8[15]= 117 ;  // 293 *******
route_copy_list[ 3 ].u8[14]= 160 ;   route_copy_list[ 3 ].u8[15]= 121 ;  // 297
route_copy_list[ 4 ].u8[14]= 181 ;   route_copy_list[ 4 ].u8[15]= 105 ;  // 298
route_copy_list[ 5 ].u8[14]= 19 ;   route_copy_list[ 5 ].u8[15]= 98 ;  // 299
route_copy_list[ 6 ].u8[14]= 136 ;   route_copy_list[ 6 ].u8[15]= 118 ;  // 300
route_copy_list[ 7 ].u8[14]= 166 ;   route_copy_list[ 7 ].u8[15]= 117 ;  // 301
route_copy_list[ 8 ].u8[14]= 163 ;   route_copy_list[ 8 ].u8[15]= 112 ;  // 303
route_copy_list[ 9 ].u8[14]= 160 ;   route_copy_list[ 9 ].u8[15]= 130 ;  // 307
route_copy_list[ 10 ].u8[14]= 195 ;   route_copy_list[ 10 ].u8[15]= 104 ;  // 309
route_copy_list[ 11 ].u8[14]= 193 ;   route_copy_list[ 11 ].u8[15]= 105 ;  // 310
route_copy_list[ 12 ].u8[14]= 165 ;   route_copy_list[ 12 ].u8[15]= 112 ;  // 311
route_copy_list[ 13 ].u8[14]= 151 ;   route_copy_list[ 13 ].u8[15]= 130 ;  // 312
route_copy_list[ 14 ].u8[14]= 144 ;   route_copy_list[ 14 ].u8[15]= 130 ;  // 313
route_copy_list[ 15 ].u8[14]= 149 ;   route_copy_list[ 15 ].u8[15]= 128 ;  // 314
route_copy_list[ 16 ].u8[14]= 178 ;   route_copy_list[ 16 ].u8[15]= 118 ;  // 317
route_copy_list[ 17 ].u8[14]= 131 ;   route_copy_list[ 17 ].u8[15]= 112 ;  // 318
route_copy_list[ 18 ].u8[14]= 152 ;   route_copy_list[ 18 ].u8[15]= 121 ;  // 321
route_copy_list[ 19 ].u8[14]= 178 ;   route_copy_list[ 19 ].u8[15]= 130 ;  // 322
route_copy_list[ 20 ].u8[14]= 167 ;   route_copy_list[ 20 ].u8[15]= 130 ;  // 324
route_copy_list[ 21 ].u8[14]= 151 ;   route_copy_list[ 21 ].u8[15]= 118 ;  // 325
route_copy_list[ 22 ].u8[14]= 146 ;   route_copy_list[ 22 ].u8[15]= 130 ;  // 328
route_copy_list[ 23 ].u8[14]= 164 ;   route_copy_list[ 23 ].u8[15]= 114 ;  // 335
route_copy_list[ 24 ].u8[14]= 178 ;   route_copy_list[ 24 ].u8[15]= 121 ;  // 337
route_copy_list[ 25 ].u8[14]= 150 ;   route_copy_list[ 25 ].u8[15]= 104 ;  // 338
route_copy_list[ 26 ].u8[14]= 148 ;   route_copy_list[ 26 ].u8[15]= 119 ;  // 339
route_copy_list[ 27 ].u8[14]= 165 ;   route_copy_list[ 27 ].u8[15]= 104 ;  // 343
route_copy_list[ 28 ].u8[14]= 192 ;   route_copy_list[ 28 ].u8[15]= 130 ;  // 344
route_copy_list[ 29 ].u8[14]= 133 ;   route_copy_list[ 29 ].u8[15]= 112 ;  // 350
//route_copy_list[ 29 ].u8[14]= 182 ;   route_copy_list[ 29 ].u8[15]= 105 ;  // 346    *******************
route_copy_list[ 30 ].u8[14]= 152 ;   route_copy_list[ 30 ].u8[15]= 104 ;  // 348
route_copy_list[ 31 ].u8[14]= 37 ;   route_copy_list[ 31 ].u8[15]= 83 ;  // 349
route_copy_list[ 32 ].u8[14]= 147 ;   route_copy_list[ 32 ].u8[15]= 103 ;  // 354
//route_copy_list[ 33 ].u8[14]= 153 ;   route_copy_list[ 33 ].u8[15]= 117 ;  // 291 *******
route_copy_list[ 33 ].u8[14]= 146 ;   route_copy_list[ 33 ].u8[15]= 103 ;  // 353
route_copy_list[ 34 ].u8[14]= 134 ;   route_copy_list[ 34 ].u8[15]= 105 ;  // 294
route_copy_list[ 35 ].u8[14]= 148 ;   route_copy_list[ 35 ].u8[15]= 103 ;  // 295
//route_copy_list[ 36 ].u8[14]= 144 ;   route_copy_list[ 36 ].u8[15]= 105 ;  // 296 *******
route_copy_list[ 36 ].u8[14]= 134 ;   route_copy_list[ 36 ].u8[15]= 114 ;  // 357
route_copy_list[ 37 ].u8[14]= 146 ;   route_copy_list[ 37 ].u8[15]= 113 ;  // 302
route_copy_list[ 38 ].u8[14]= 136 ;   route_copy_list[ 38 ].u8[15]= 119 ;  // 304
route_copy_list[ 39 ].u8[14]= 136 ;   route_copy_list[ 39 ].u8[15]= 113 ;  // 305
route_copy_list[ 40 ].u8[14]= 148 ;   route_copy_list[ 40 ].u8[15]= 121 ;  // 306
route_copy_list[ 41 ].u8[14]= 176 ;   route_copy_list[ 41 ].u8[15]= 131 ;  // 308
//route_copy_list[ 42 ].u8[14]= 136 ;   route_copy_list[ 42 ].u8[15]= 130 ;  // 315 *******
route_copy_list[ 42 ].u8[14]= 180 ;   route_copy_list[ 42 ].u8[15]= 121 ;  // 355
route_copy_list[ 43 ].u8[14]= 8 ;   route_copy_list[ 43 ].u8[15]= 98 ;  // 316
route_copy_list[ 44 ].u8[14]= 165 ;   route_copy_list[ 44 ].u8[15]= 119 ;  // 319
route_copy_list[ 45 ].u8[14]= 164 ;   route_copy_list[ 45 ].u8[15]= 119 ;  // 320
route_copy_list[ 46 ].u8[14]= 167 ;   route_copy_list[ 46 ].u8[15]= 119 ;  // 323
route_copy_list[ 47 ].u8[14]= 164 ;   route_copy_list[ 47 ].u8[15]= 128 ;  // 326
route_copy_list[ 48 ].u8[14]= 9 ;   route_copy_list[ 48 ].u8[15]= 98 ;  // 327
route_copy_list[ 49 ].u8[14]= 177 ;   route_copy_list[ 49 ].u8[15]= 119 ;  // 329
route_copy_list[ 50 ].u8[14]= 176 ;   route_copy_list[ 50 ].u8[15]= 130 ;  // 330
//route_copy_list[ 51 ].u8[14]= 162 ;   route_copy_list[ 51 ].u8[15]= 130 ;  // 331 *******
route_copy_list[ 51 ].u8[14]= 166 ;   route_copy_list[ 51 ].u8[15]= 129 ;  // 356
route_copy_list[ 52 ].u8[14]= 133 ;   route_copy_list[ 52 ].u8[15]= 120 ;  // 332
route_copy_list[ 53 ].u8[14]= 161 ;   route_copy_list[ 53 ].u8[15]= 120 ;  // 333
route_copy_list[ 54 ].u8[14]= 183 ;   route_copy_list[ 54 ].u8[15]= 130 ;  // 334
route_copy_list[ 55 ].u8[14]= 148 ;   route_copy_list[ 55 ].u8[15]= 104 ;  // 336
route_copy_list[ 56 ].u8[14]= 164 ;   route_copy_list[ 56 ].u8[15]= 121 ;  // 340
route_copy_list[ 57 ].u8[14]= 185 ;   route_copy_list[ 57 ].u8[15]= 131 ;  // 341
route_copy_list[ 58 ].u8[14]= 176 ;   route_copy_list[ 58 ].u8[15]= 121 ;  // 342
route_copy_list[ 59 ].u8[14]= 145 ;   route_copy_list[ 59 ].u8[15]= 103 ;  // 345
route_copy_list[ 60 ].u8[14]= 179 ;   route_copy_list[ 60 ].u8[15]= 104 ;  // 347

//route_copy_list[ 63 ].u8[14]= 136 ;   route_copy_list[ 63 ].u8[15]= 112 ;  // 352 ******

    for ( i=0;i<num_child;i++){
       printf("%d: %u.%u.", i, route_copy_list[i].u8[14], route_copy_list[i].u8[15]);
    }
    printf("\n");
    
  }

#if DO_SEQ_SEND_WITH_INTERVAL
//    printf("do_seq_send_with_interval==1\n");
#else
//    printf("do_seq_send_with_interval==0\n");
    while(num_child > 0){
        send_packet(&route_copy_list[num_child-1]);
    }    
#endif

  PRINTF("------------- SEND PROCESS END----------------------------\n");
  print_mac_states();




}




static void
send_process1()
{
   PRINTF("-------------SEND PROCESS START ! (MAKE DOWNSTREAM DESTINATION LIST)\n");
    uip_ds6_route_t *route;
    route = uip_ds6_route_head();


    num_child=0;


    while(route != NULL && num_child < MAX_NODE_NUM) {
      PRINTF("-- %u %u --- ", route->ipaddr.u8[sizeof(route->ipaddr.u8)-2], route->ipaddr.u8[sizeof(route->ipaddr.u8)-1]);  //      PRINT6ADDR(&route->ipaddr);
      uip_ipaddr_copy(&route_copy_list[num_child], &route->ipaddr);
	int i;
	for ( i=0;i<sizeof(route_copy_list->u8);i++){
	 printf("%u.", route_copy_list[num_child].u8[i]);
	}
	printf("\n");

      num_child++;

      route = uip_ds6_route_next(route);
    }


#if DO_SEQ_SEND_WITH_INTERVAL
//    printf("do_seq_send_with_interval==1\n");
#else
//    printf("do_seq_send_with_interval==0\n");
    while(num_child > 0){
        send_packet(&route_copy_list[num_child-1]);
    }    
#endif

  PRINTF("------------- SEND PROCESS END----------------------------\n");
  print_mac_states();

}

#endif

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *appdata;


  if(uip_newdata()) {
    appdata = (char *)uip_appdata;
    appdata[uip_datalen()] = 0;
    PRINTF("D rxvs %s fr %u %u t %u\n", appdata, UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 2], UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1], (uint16_t) current_asn.ls4b);  //modified


#if SERVER_REPLY
#if ECHO_DOWNSTREAM_ENABLED
    send_packet(&UIP_IP_BUF->srcipaddr);

    print_mac_states();

#endif
#endif



  }
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
net_init(uip_ipaddr_t *br_prefix)
{
  uip_ipaddr_t global_ipaddr;

  if(br_prefix) { /* We are RPL root. Will be set automatically
                     as TSCH pan coordinator via the tsch-rpl module */
    memcpy(&global_ipaddr, br_prefix, 16);
    uip_ds6_set_addr_iid(&global_ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&global_ipaddr, 0, ADDR_AUTOCONF);
    rpl_set_root(RPL_DEFAULT_INSTANCE, &global_ipaddr);
    rpl_set_prefix(rpl_get_any_dag(), br_prefix, 64);
    rpl_repair_root(RPL_DEFAULT_INSTANCE);
  }

  NETSTACK_MAC.on();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{

  static struct etimer et;

#if !ECHO_DOWNSTREAM_ENABLED
  static struct etimer periodic;

#if DO_SEQ_SEND_WITH_INTERVAL
  static struct etimer sequential_send;

#endif
#endif

  static struct etimer startAfter;



  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;

  PROCESS_BEGIN();
#if WITH_COMPOWER & SERVER_WITH_COMPOWER
  powertrace_start(CLOCK_SECOND*60);
#endif
  PROCESS_PAUSE();

  SENSORS_ACTIVATE(button_sensor);

  PRINTF("UDP server started. max nbr:%d max routes:%d\n",
         NBR_TABLE_CONF_MAX_NEIGHBORS, UIP_CONF_MAX_ROUTES);


#if UIP_CONF_ROUTER

  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */
  
  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high 
     packet reception rates. */
//  NETSTACK_MAC.off(1); //ksh..










 /* 3 possible roles:
   * - role_6ln: simple node, will join any network, secured or not
   * - role_6dr: DAG root, will advertise (unsecured) beacons
   * - role_6dr_sec: DAG root, will advertise secured beacons
   * */
  static int is_coordinator = 0;
  static enum { role_6ln, role_6dr, role_6dr_sec } node_role;
  node_role = role_6dr;//dr;

  int coordinator_candidate = 0;

#ifdef CONTIKI_TARGET_Z1
  /* Set node with MAC address c1:0c:00:00:00:00:01 as coordinator,
   * convenient in cooja for regression tests using z1 nodes
   * */
  extern unsigned char node_mac[8];
  unsigned char coordinator_mac[8] = { 0xc1, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };

  coordinator_candidate = (memcmp(node_mac, coordinator_mac, 8) == 0);
#elif CONTIKI_TARGET_COOJA
  //coordinator_candidate = (node_id == 1);
#endif

  if(coordinator_candidate) {
    if(LLSEC802154_ENABLED) {
      node_role = role_6dr_sec;
    } else {
      node_role = role_6dr;
    }
  } else {
    node_role = role_6dr;
  }

#if CONFIG_VIA_BUTTON
  {
#define CONFIG_WAIT_TIME 5

    SENSORS_ACTIVATE(button_sensor);
    etimer_set(&et, CLOCK_SECOND * CONFIG_WAIT_TIME);





    while(!etimer_expired(&et)) {
      printf("Init: current role: %s. Will start in %u seconds. Press user button to toggle mode.\n",
             node_role == role_6ln ? "6ln" : (node_role == role_6dr) ? "6dr" : "6dr-sec",
             CONFIG_WAIT_TIME);
      PROCESS_WAIT_EVENT_UNTIL(((ev == sensors_event) &&
                                (data == &button_sensor) && button_sensor.value(0) > 0)
                               || etimer_expired(&et));
      if(ev == sensors_event && data == &button_sensor && button_sensor.value(0) > 0) {
        node_role = (node_role + 1) % 3;
        if(LLSEC802154_ENABLED == 0 && node_role == role_6dr_sec) {
          node_role = (node_role + 1) % 3;
        }
        etimer_restart(&et);
      }
    }
  }

#endif /* CONFIG_VIA_BUTTON */

  printf("Init: node starting with role %s\n",
         node_role == role_6ln ? "6ln" : (node_role == role_6dr) ? "6dr" : "6dr-sec");

  tsch_set_pan_secured(LLSEC802154_ENABLED && (node_role == role_6dr_sec));
  is_coordinator = node_role > role_6ln;

  if(is_coordinator) {
    uip_ipaddr_t prefix;
    uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    net_init(&prefix);
  } else {
    net_init(NULL);
  }

#if WITH_ORCHESTRA
  orchestra_init();
#endif /* WITH_ORCHESTRA */

  etimer_set(&startAfter, START_AFTER_PERIOD);

#if !ECHO_DOWNSTREAM_ENABLED
  etimer_set(&periodic, SEND_INTERVAL_SERVER);

#if DO_SEQ_SEND_WITH_INTERVAL
  etimer_set(&sequential_send, SEQUENTIAL_SEND_INTERVAL);

#endif
#endif

  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
  if(server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));



  PRINTF("======================================= \n");
  PRINTF(" INFOMATION OF EVALUATIONAL SETTINGS \n");
  PRINTF(" ORCHESTRA_CONF_COMMON_SHARED_PERIOD: %d\n", ORCHESTRA_CONF_COMMON_SHARED_PERIOD);
  PRINTF(" ORCHESTRA_CONF_UNICAST_PERIOD: %d\n", ORCHESTRA_CONF_UNICAST_PERIOD);
  PRINTF(" PERIOD: %d\n", PERIOD);
  PRINTF(" SEND_INTERVAL: %d\n", SEND_INTERVAL);
  PRINTF(" SEND_INTERVAL_SERVER: %d\n", SEND_INTERVAL_SERVER);
  PRINTF(" SEQUENTIAL_SEND_INTERVAL: %d\n", SEQUENTIAL_SEND_INTERVAL);
  PRINTF(" WITH_COMPOWER: %d\n", WITH_COMPOWER);
  PRINTF(" ECHO_DOWNSTREAM_ENABLED: %d\n", ECHO_DOWNSTREAM_ENABLED);
  PRINTF(" DO_SEQ_SEND_WITH_INTERVAL: %d\n", DO_SEQ_SEND_WITH_INTERVAL);
  PRINTF(" FIXED_RPL_TOPOLOGY: %d\n", FIXED_RPL_TOPOLOGY);
  PRINTF(" WITH_ALICE: %d\n", WITH_ALICE);
  PRINTF(" ORCHESTRA_CONF_UNICAST_SENDER_BASED: %d\n", ORCHESTRA_CONF_UNICAST_SENDER_BASED);
  PRINTF(" ORCHESTRA_UNICAST_SENDER_BASED: %d\n", ORCHESTRA_UNICAST_SENDER_BASED);
  PRINTF(" QUEUEBUF_NUM: %d\n", QUEUEBUF_NUM);
  PRINTF(" TSCH_QUEUE_MAX_NEIGHBOR_QUEUES: %d\n", TSCH_QUEUE_MAX_NEIGHBOR_QUEUES);

  PRINTF("======================================= \n");



  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiaing global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    } 

   if(etimer_expired(&startAfter)) {
#if !ECHO_DOWNSTREAM_ENABLED
     if(etimer_expired(&periodic)) {
       etimer_restart(&periodic);
       send_process1();
     }
#if DO_SEQ_SEND_WITH_INTERVAL
     if(etimer_expired(&sequential_send)){
       etimer_restart(&sequential_send);
       if(num_child>0) {        
      	 send_packet(&route_copy_list[num_child-1]);
       }
     }
#endif
#endif
   }//startAfter


  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
