/*
 * beaglefetch.c - Main code running on the Beaglebone Black.  This code acts 
 * as a TCP server that receives motor velocity commands from Rockie's main 
 * computer.
 *
 * RPI Rock Raiders
 * 4/21/14
 *
 * Last Updated: Bryant Pong: 4/23/14 - 5:56 PM
 */     

// STL Libraries:
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

// Constants
#define BUFFER_SIZE 1024

#define GPIO1_START_ADDR
#define GPIO2_START_ADDR

#define GPIO1_END_ADDR
#define GPIO2_END_ADDR

#define GPIO1_SIZE (GPIO1_END_ADDR - GPIO1_START_ADDR)
#define GPIO2_SIZE (GPIO2_END_ADDR - GPIO2_START_ADDR)

#define GPIO1_OE 0x134
#define GPIO2_OE

#define GPIO1_SETDATAOUT
#define GPIO2_SETDATAOUT

#define GPIO1_CLEARDATAOUT
#define GPIO2_CLEARDATAOUT 

#define PIN1
#define PIN2
  
// End Section Constants

// Function Prototypes
// End Function Prototypes


// Main function:
int main(int argc, char **argv) {

	/*
	 * Beaglebone Hardware Registers:
	 */

	

	volatile void *gpio1_addr = NULL;
	volatile void *gpio2_addr = NULL;

	volatile unsigned int *gpio1_oe_addr = NULL;
	volatile unsigned int *gpio2_oe_addr = NULL;

	volatile unsigned int *gpio1_setdataout_addr = NULL;
	volatile unsigned int *gpio2_setdataout_addr = NULL;

	volatile unsigned *gpio1_cleardataout_addr = NULL;
	volatile unsigned *gpio2_cleardataout_addr = NULL; 
		 
	unsigned int reg1, reg2;

		

	// Buffer to receive messages from Rockie:
	char buffer[BUFFER_SIZE];

	// Socket Structures for socket:
	struct sockaddr_in beagle, rockie; 

	// The Beaglebone is listening on 192.168.7.2:9001:
	unsigned short port = 9001;

	// Socket to listen to Rockie:
	int sock = socket(PF_INET, SOCK_STREAM, 0);

	if(sock < 0) {
		fprintf(stderr, "Error in main(): cannot create Rockie socket!\n");
		return EXIT_FAILURE;
	} // End if
	
	// Populate the server structs:
	beagle.sin_family = PF_INET;
	beagle.sin_addr.s_addr = INADDR_ANY;
	beagle.sin_port = htons(port);			 
	int len = sizeof(beagle);

	// Bind the struct to the listener socket:
	if(bind(sock, (struct sockaddr *) &beagle, len) < 0) {
		fprintf(stderr, "Error in main(): cannot bind Beagle socket!\n");
		return EXIT_FAILURE;
	} // End if

	unsigned int fromlen = sizeof(rockie);
	// Listen on just 1 client (Rockie):
	listen(sock, 1);

	int newsock = accept(sock, (struct sockaddr *) &rockie, &fromlen);
	printf("Accepted client connection\n");

	while(1) {
		//int newsock = accept(sock, (struct sockaddr *) &rockie, &fromlen);
		//printf("Accepted client connection\n");
		//fflush(NULL);

		int n = recv(newsock, buffer, BUFFER_SIZE - 1, 0);
		buffer[n] = '\0';
		printf("Received message from %s: %s\n",
				inet_ntoa( (struct in_addr) rockie.sin_addr), buffer);

		n = send(newsock, buffer, strlen(buffer), 0);
		if(n < strlen(buffer)) {
			fprintf(stderr, "Error in main(): sending echo message failed\n");
			return EXIT_FAILURE;
		} else {
			printf("Sending %s to client\n", buffer);
		} // End else 	
	} // End while

	close(newsock);

	return 0;
} // End function main()
