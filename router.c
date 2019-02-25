#include "contiki.h"
#include "net/rime.h"
#include <stdio.h> /* For printf() */
#include "cc2430_sfr.h"

#define TABLELENGTH        10

#define COMMAND_ROUTREQUEST   0x20	//Command for requesting route
#define COMMAND_ROUTERESPONSE 0x21	//Command for route response
#define COMMAND_DTATTX        0x22	//Command for sending data through unicast

#define BATTERY_AVERGE_LVL    3000


typedef struct	//group of data elements grouped together under one name
{
    uint16_t fDest;
    uint16_t nextHop;
    uint16_t origin;
    uint16_t count;
} rFwdTable;

typedef struct	//group of data elements grouped together under one name
{
    uint16_t bDest;
    uint16_t nextHop;
    uint16_t origin;
    uint16_t count;
} rBwdTable;

static rBwdTable bwdTable;
static rFwdTable fwdTable;

static rimeaddr_t addr;
static uint8_t destination;
static struct etimer et;
static struct unicast_conn uc;
static struct broadcast_conn bc;    //broadcast_conn struct
static const struct broadcast_callbacks broadcast_callbacks = {recv_bc};    //Register the callback routine
static const struct unicast_callbacks unicast_callbacks = {recv_uc};

static uint8_t u8DataBuffer[50];    //create my buffer
static int i = 0;
static uint16_t myAddress=0xAA16;
static uint16_t addressbook[6] = {0xAA11,0xAA12,0xAA13,0xAA14,0xAA15,0xAA16};

static uint8_t name = 6;

//static char letter_name = "F";

uint16_t dest=0;
uint16_t origin=0;
uint16_t source=0;

static long path;
static uint8_t nonc;

uint8_t path_arr[6];
uint8_t num = 0;
uint8_t count = 0;
uint8_t counter = 0;
uint8_t y = 0;
uint8_t u;
uint8_t checker = 0;
uint16_t battery =0;




/*---------------------------------------------------------------------------*/
PROCESS(rf_test_process, "RF test RX process"); //declare a process  called "rf_test_process" and a string to identify it
AUTOSTART_PROCESSES(&rf_test_process);  //defines the process which will be loaded when the system starts up i.e rf_test_process
/*---------------------------------------------------------------------------*/

static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
    uint8_t * data;

	data = packetbuf_dataptr();
    //unsigned int atDest=0;
    //uint16_t source=0;
	
    switch(data[0])
    {
        case COMMAND_ROUTERESPONSE:
			printf("\n Got a request uc \n\r");
            //update forward TABLE
            //look up nexthop in in backwards table and send there

            //get the destination
            dest = data[1];
            dest = dest << 8;
            dest = dest | data[2];

            //get the origin of the packet
            origin = data[6];
            origin = origin << 8;
            origin = origin | data[7];

            source = from->u8[0];
            source = source <<8;
            source = source | from->u8[1];

            fwdTable.fDest = dest;
            fwdTable.nextHop = source;
            //fwdTable[i].origin = origin;
            fwdTable.count = data[3];
            //maybe increment


            u8DataBuffer[0] = COMMAND_ROUTERESPONSE;
            u8DataBuffer[1] = data[1];
            u8DataBuffer[2] = data[2];
            u8DataBuffer[3] = data[3];
            u8DataBuffer[4] = data[4];
            u8DataBuffer[5] = data[5];
            u8DataBuffer[6] = data[6];
            u8DataBuffer[7] = data[7];
			u8DataBuffer[8] = data[8];
			u8DataBuffer[9] = data[9];
			u8DataBuffer[10] = data[10];
			u8DataBuffer[11] = data[11];
			u8DataBuffer[12] = data[12];
            //strcpy(u8DataBuffer[8] ,route);
            //add battery level

            packetbuf_copyfrom(u8DataBuffer,13);
            addr.u8[1] = bwdTable.nextHop;
            addr.u8[0] = bwdTable.nextHop>>8;
            printf("\n next %02x%02x \n\r",addr.u8[1],addr.u8[0]);
            unicast_send(&uc, &addr);
		
		case COMMAND_DTATTX:
		printf("\n Got a transmission \n\r");
		printf(" \n Temp = %d.%02u C \n\r", data[3], data[4]);
		printf(" \n  battery = %d %d\n\r",data[6], data[5]);
		
			if(data[2] != name){
			printf("\n Got a data msg\n\r");
			packetbuf_copyfrom(data,7);
			addr.u8[1] = fwdTable.nextHop;
            addr.u8[0] = fwdTable.nextHop>>8;
			unicast_send(&uc, &addr);
			}
			if(data[2] == name){
				
				printf("\n msg from sender: %s\n\r",data[1]);
				
			}
		
        default:
            break;
    }
}

//define callback function for broadcast reception
static void
recv_bc(struct broadcast_conn *c, rimeaddr_t *from)
{
	uint8_t * data;
	uint8_t n;
	
	data = packetbuf_dataptr();
	n =  packetbuf_datalen();



	//uint16_t dest=0;
	//uint16_t origin=0;
	//uint16_t source=0;

	switch(data[0])
    {

        case COMMAND_ROUTREQUEST:
			printf("\n Got a request bc: %d \n\r",data[n-1]);
			checker = 0;
			// get the path
			//path = data[8];


            //get the destination
            dest = data[1];
            dest = dest << 8;
            dest = dest | data[2];

            //get the origin of the packet
            origin = data[6];
            origin = origin << 8;
            origin = origin | data[7];

            //get the next hop (backwards)
            source = from->u8[0];
            source = source <<8;
            source = source | from->u8[1];

			for ( y = 0;y < 6;y++){	
				if(source == addressbook[y]){
					checker = 1;
				}
			}
			/* printf("the checker: %d\n",checker);
			printf("the source: %02x\n",source);
			printf("the destination: %02x\n",dest);
			printf("the my addr: %02x\n",myAddress); */
            if((myAddress != dest)&&(checker == 1))   //if not at the destination
            {
				printf('go in 1');
                if((bwdTable.bDest != origin) && (bwdTable.nextHop != source)) //if not already recieved a packet
                {
					
					printf("\n changing the current reverse tbl \n\r");
					
					printf("\n n: %d \n\r",n);
					data[n] = name;
					
					
					bwdTable.bDest = origin;
                    bwdTable.nextHop = source;
                    //bwdTable.origin = origin;
                    bwdTable.count = data[3];
                    //rebroadcast
                    packetbuf_copyfrom(data, n+1);	//copy some stuff from buffer and broadcast it

					broadcast_send(&bc);
                }
				data[n] = name;
				packetbuf_copyfrom(data, n+1);
				broadcast_send(&bc);
            }

            else{   //if at destation
				
				
				
				
				printf("\n the n: %d \n\r",n);
				printf("\n I am dest \n the path: \n\r");
				
				for(u = 8; u < n ; u++){
					
					printf("%d-",data[u]);
					
				}
				printf("%d",name);
				packetbuf_clear();
				

                //update back table
                bwdTable.bDest = origin;
                bwdTable.nextHop = source;
                //bwdTable[i].origin = origin;
                bwdTable.count = data[3];

                //update forward table
                fwdTable.fDest = dest;
                fwdTable.nextHop = dest;
                fwdTable.origin = origin;
                //fwdTable.count = data[3];
				
                //send a unicast message to the next hop in reverse TABLE
                //unicast message must be RREP
                u8DataBuffer[0] = COMMAND_ROUTERESPONSE;
                u8DataBuffer[1] = data[1];
				u8DataBuffer[2] = data[2];
				u8DataBuffer[3] = data[3];
				u8DataBuffer[4] = data[4];
				u8DataBuffer[5] = data[5];
				u8DataBuffer[6] = data[6];
				u8DataBuffer[7] = data[7];
				//strcpy(u8DataBuffer[8] ,route);
				//u8DataBuffer[8] = nonc;
               // u8DataBuffer[8] = route;
				//u8DataBuffer[9] = data[9];
					//u8DataBuffer[10] = data[10];
					//u8DataBuffer[11] = data[11];
					//u8DataBuffer[12] = data[12];
                //battery lvl goes here in buffer

                packetbuf_copyfrom(u8DataBuffer,8);
                addr.u8[1] = bwdTable.nextHop;
                addr.u8[0] = bwdTable.nextHop>>8;
                printf("\n next %02x%02x \n\r",addr.u8[1],addr.u8[0]);
                unicast_send(&uc, &addr);

            }

            default:
    			break;
        }
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rf_test_process, ev, data) //define the body of the thread
{
    PROCESS_BEGIN(); //begin the process
    printf("\nStarting CC2430 RF test suite...\n\r");

    //initialise tables

	for (i=5;i>=0;i--){
		path_arr[i] = 0;

	}

        bwdTable.bDest = 0x0000;
        bwdTable.nextHop = 0xffff;
        bwdTable.origin = 0xffff;
        bwdTable.count = 0;

        fwdTable.fDest = 0x0000;
        fwdTable.nextHop = 0xffff;
        fwdTable.origin = 0xffff;
        fwdTable.count = 0;


    broadcast_open(&bc, 128, &broadcast_callbacks); //Open the channel for broadcast (bc = struct for broadcast, 128 = channel, )
                                                    //A struct broadcast_callbacks with function pointers to functions that will be called when a packet has been received
    unicast_open(&uc, 129, &unicast_callbacks);	//set up unicasting

    etimer_set(&et, CLOCK_SECOND);  //Setup an event timer to let a process execute once per second.
    while(1) {
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));  //wait until timer expires
        etimer_reset(&et);  //reset the timer
    }
             PROCESS_END();
}
/*---------------------------------------------------------------------------*/
