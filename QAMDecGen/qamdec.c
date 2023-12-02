/*
* qamdec.c
*
* Created: 05.05.2020 16:38:25
*  Author: Chaos
*/ 

#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "stack_macros.h"

#include "mem_check.h"
#include "errorHandler.h"

#include "qaminit.h"
#include "qamdec.h"

QueueHandle_t decoderQueue;
uint8_t receivebuffer[100] = {1, 3, 1, 0, 0, 1, 0, 0, 3, 3,	1, 2, 0, 2, 2, 0, 0, 1, 1, 2, 1, 0,0 ,0, 2, 1, 0, 0, 0, 0, 0, 0};
uint8_t symbol = 0;
uint8_t checksumGL = 0; // Initialisierung der Checksumme
uint8_t calculatedChecksum = 0; // Variable für die berechnete Checksumme
int k = 0;
int time_since_pik = 0;
float reconstructedFloat;



// Funktion zur Generierung des Signals basierend auf der Zeit seit dem letzten "Pik"
// Wie?

char generate_signal(int time_since_last_pik) {
	if (time_since_last_pik < 32) {
		return '0';
		} else if (time_since_last_pik < 40) {
		return '1';
		} else if (time_since_last_pik < 48) {
		return '2';
		} else {
		return '3';
	}
}



void vQuamDec(void* pvParameters)
{
	( void ) pvParameters;
	unsigned char byteArray[4];
	byteArray[0] = 0b00000000;
	byteArray[1] = 0b00000000;
	byteArray[2] = 0b10110110;
	byteArray[3] = 0b01000001;
	decoderQueue = xQueueCreate( 4, NR_OF_SAMPLES * sizeof(int16_t) );
	char calculatedChecksum = 0;
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
	
	uint16_t bufferelement[NR_OF_SAMPLES];
	
	xEventGroupWaitBits(evDMAState, DMADECREADY, false, true, portMAX_DELAY);
	for(;;) {
		// Annahme: Hier wird die Zeit seit dem letzten "Pik"-Signal aktualisiert
		time_since_pik++; // Aktualisiere die Zeit bei jedem Schleifendurchlauf
		
		while(uxQueueMessagesWaiting(decoderQueue) > 0) {
			if(xQueueReceive(decoderQueue, &bufferelement[0], portMAX_DELAY) == pdTRUE) {
				//Decode Buffer
					for (size_t i = 0; i < (NR_OF_SAMPLES-8); ++i) {
						calculatedChecksum += receivebuffer[i];
					}
				
					for (k = 0; k < NR_OF_SAMPLES -8; k++){
					 symbol = generate_signal(time_since_pik); // Generiert das Signal entsprechend der Zeit seit dem letzten "Pik"-Signal
					// Decodiere das Symbol
					switch (symbol){
						case '0':
							checksumGL += receivebuffer[k];
						break;
						case '1':
							checksumGL += receivebuffer[k];
						break;
						case '2':
							checksumGL += receivebuffer[k];
						break;
						case '3':
							checksumGL += receivebuffer[k];
						break;
						default:
							// Auf Dispaly "Unknown symbol received!"
						break;
					}
					

					// Zurücksetzen des Zählers nach jedem Signal
					time_since_pik = 0;
					

				}
				if (calculatedChecksum == checksumGL) {
					memcpy(&reconstructedFloat, byteArray, sizeof(float));
				}
				
				calculatedChecksum = 0;
				checksumGL = 0;
			}
		}		
		vTaskDelay( 2 / portTICK_RATE_MS );
	}
}

void fillDecoderQueue(uint16_t buffer[NR_OF_SAMPLES])
{
	BaseType_t xTaskWokenByReceive = pdFALSE;

	xQueueSendFromISR( decoderQueue, &buffer[0], &xTaskWokenByReceive );
}

ISR(DMA_CH2_vect)
{
	DMA.CH2.CTRLB|=0x10;

	fillDecoderQueue( &adcBuffer0[0] );
}

ISR(DMA_CH3_vect)
{
	DMA.CH3.CTRLB |= 0x10;

	fillDecoderQueue( &adcBuffer1[0] );
}

/*

unsigned char byteArray[4];

    // Hier füge den Code zum Einlesen des Byte-Arrays ein
    // Du kannst das Byte-Array von einer Datei lesen, von der Benutzereingabe oder anderweitig erhalten

    // Beispiel: Annahme, dass das Byte-Array bereits vorliegt
    // Beachte, dass dies nur ein Beispiel ist und die genaue Art des Einlesens von deiner Anwendung abhängt
    // Möglicherweise möchtest du dies aus einer Datei oder von einem anderen Ort lesen.
    for (int i = 0; i < sizeof(byteArray); ++i) {
        // Hier wird das Byte-Array mit Beispieldaten initialisiert
        byteArray[0] = 0b10110101;
        byteArray[1] = 0b01101000;
        byteArray[2] = 0b10010110;
        byteArray[3] = 0b01000010;
    }

    // Umwandlung des Byte-Arrays zurück in einen Float
    float reconstructedFloat;
    memcpy(&reconstructedFloat, byteArray, sizeof(float));

    // Ausgabe des rekonstruierten Float-Werts
    printf("Rekonstruierter Float-Wert: %f\n", reconstructedFloat);

    return 0;
}

*/