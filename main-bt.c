/*
    Title: CMPE460 Lab4 - Bluetooth Low Energy 
    Description: UART driver to send and receive characters
    Authors: Zeb Hollinger
	Louis J. Beato
	3/1/2021
*/

#include "msp.h"
#include "uart.h"         // Main serial funcitonality
#include "led.h"          // OPTIONAL: Could be useful to display info/warnings
#include <stdio.h>        // General funcitonality
#include <string.h>       // Useful for string operations and memset

#define CHAR_COUNT 80     // Size of a console line

int main(){
	
	char ch;
	char phoneBuffer[CHAR_COUNT];
	char *tempPhonePtr;
	char *phonePtr;
	char pcBuffer[CHAR_COUNT];
	char *tempPcPtr;
	char *pcPtr;
	
	// Initialize LEDs (optional)
	LED1_Init(); // might not even be needed
	LED2_Init();
	
	// Initialize UART0 and UART2
	uart0_init();
	uart2_init();
	
	// Display startup message
	uart0_put("\r\nChoose a Chat Room to join.");   /*Transmit this through UART0*/
	uart0_put("\r\nChat Room A selected!");    			/*Transmit this through UART0*/
	uart2_put("\r\nChoose a Chat Room to join.");   /*Transmit this through UART2*/
	uart2_put("\r\nChat Room A selected!");    			/*Transmit this through UART2*/


	
	// Declare and reset buffers and print prompts
	memset(phoneBuffer, 0, sizeof phoneBuffer);
	phonePtr = &phoneBuffer[0];
	memset(pcBuffer, 0, sizeof pcBuffer);
	pcPtr = &pcBuffer[0];
	uart0_put("\r\nPC>");
	uart2_put("\r\nPhone>");


	
	/* Control loop */
	while(1){
		
		/* IF PC SENT SOMETHING */
		if (uart0_dataAvailable() == TRUE){

			LED1_On();
			// Retrieve the character
			ch = uart0_getchar(); // get the character
			
		
			/* If statements to handle the character */
			if (ch == '\n' || ch == '\r'){
				/** 
				 * Enter pressed. Phone terminal may not \r or \n
				 * Manual entering of a character may be needed
				 * Some terminals enter \r then \n. Handle this.
				 */
				 
				/* Handle text coming from the PC */ 
			
				
				/*     ON THE PHONE SIDE    */
				// Clear current text
				uart2_put("\r                                                          ");				
				
				// Print the PC's message with a newline
				uart2_put("PC>");
				tempPcPtr = &pcBuffer[0];
				while(tempPcPtr < pcPtr){
					ch = *tempPcPtr;
					uart2_putchar(ch);
						
					tempPcPtr++;
				}	
				uart2_putchar('\n');
				
				// Restore the current text
				tempPhonePtr = &phoneBuffer[0];
				uart2_put("Phone>");
				while(tempPhonePtr < phonePtr){
					ch = *tempPhonePtr;
					uart2_putchar(ch);
					tempPhonePtr++;
				}
				// reset PHONE buffer after popping stored text to the terminal
				// ******phone pointer still points at current spot
				
				/*      ON THE PC SIDE      */
				// Clear buffer
				// reset PC buffer after printing out to terminal
				memset(pcBuffer, 0, sizeof pcBuffer);
				pcPtr = &pcBuffer[0];

				// Newline and prompt
				uart0_put("\r\nPC>");			
					
			} 
			else if (ch == 0x7F || ch == '\b'){
				/* 
				 * Backspace detected. Some terminals don't use \b character
				 * Some enter DEL. Use an ASCII table & debugger to figure out 
				 * the correct character to check for
				*/
			
				/* Handle backspace */
				
				// Check for buffer underflow
				if (pcPtr > &pcBuffer[0]){
					uart0_putchar(ch);
					pcPtr--;
				}
			} 
			else {
							
				// Check for buffer overflow AKA buffer is full now
				if (pcPtr-&pcBuffer[0] < CHAR_COUNT){
					/* Non-special character entered */
					uart0_putchar(ch);
					
					// append character to string
					*pcPtr = ch;
					pcPtr++;
				}
			} 
		}
		/* IF PHONE SENT SOMETHING */
		else if (uart2_dataAvailable() == TRUE){
			
			LED1_On();
			// Retrieve the character
			ch = uart2_getchar();
			
			// If enter is pressed
			if (ch == '\n' || ch == '\r'){
				
				/*       ON THE PC SIDE       */
				// clear current PC line
				uart0_put("\r                                                          \r");
				
				// Print the Phone's message with a newline
				uart0_put("Phone>");
				tempPhonePtr = &phoneBuffer[0];
				while(tempPhonePtr < phonePtr){
					ch = *tempPhonePtr;
					uart0_putchar(ch);
					tempPhonePtr++;
				}
				
								
				// Restore the current PC text
				tempPcPtr = &pcBuffer[0];
				uart0_put("\r\nPC>");
				while(tempPcPtr < pcPtr){
					ch = *tempPcPtr;
					uart0_putchar(ch);
					tempPcPtr++;
				}		
				// reset PC buffer after popping stored text to terminal
				// ******PC pointer still points at current spot
				
				/*       ON THE PHONE SIDE       */
				memset(phoneBuffer, 0, sizeof phoneBuffer);
				phonePtr = &phoneBuffer[0];

				// Newline and prompt
				uart2_put("\r\nPhone>");	
				
			}
			// Backspace is pressed
			else if (ch == 0x7f || ch == '\b'){
				if (phonePtr > &phoneBuffer[0]){
						uart2_putchar(ch);
						phonePtr--;
					}
			}
			// Any other character is entered
			else {
								
				if (phonePtr-&phoneBuffer[0] < CHAR_COUNT){
					uart2_putchar(ch);
					*phonePtr = ch;
					phonePtr++;
					
				}
			}
		}
		
		else
			LED1_Off();
		
		/*  
				Repeat the logic for handling PC characters, for the Phone characters
		
				Separate non-blocking if statements function as different tasks
				This is a simple form of multi-tasking & task scheduling.
				Useful for simple polling opperations. 
				We will learn how to handle this more effciently in later labs. 
		*/
	}   
}  
