// gaga2515 (Alexandre Gagnon)
// garp2405 (Philippe Garneau)
// Ne pas voler nos trucs

#include <mbed.h>
#include <rtos.h>
#include <time.h>

DigitalOut myLed(LED1);
DigitalIn numericInput1(p15);
DigitalIn numericInput2(p16);
AnalogIn analogInput1(p19);
AnalogIn analogInput2(p20);

// Structure pour definir quel DigitalIn que les threads de lecture numerique vont lire
typedef struct {
	DigitalIn inputSource;
	uint8_t pin;
} numericSource_t;

// Structure d'un evenement
typedef struct {
    float value;
	uint8_t type;
	uint8_t pin;
    struct tm *timestamp;
} event_t;

// Structure que le RTC utilise pour communiquer
typedef struct {
	struct tm *timestamp;
	bool isValid;
} mail_t;
	
// Preparation de variables globales pour queue
MemoryPool<event_t, 50> mpool;
Queue<event_t, 50> queue;

// Preparation de la mailbox pour rtc
Mail<mail_t, 1> mailbox;

// Initialization de la semaphore pour RTC
Semaphore semaphore;

void sendEvent(float value, struct tm *date, uint8_t type, uint8_t pin) {
	// Envoi un evenement dans la queue
	// value: valeur de la variation en numerique ou valeur de la moyenne calculee en analogique
	// date: structure contenant l'information de la date et heure
	// type: analogique ou numerique
	// pin: numero de la pin pour avoir un affichage plus beau a la collection
    event_t *event = mpool.alloc();
    event->value = value;
    event->timestamp = date;
	event->type = type;
	event->pin = pin;
	
    queue.put(event);
}

bool verify(DigitalIn inputSource, uint8_t value) {
	// Verification de la stabilite du signal
	// Retourne vrai si valide sinon faux
    Thread::wait(50);
    if (inputSource.read() == value) {
        return true;
    }
    
    return false;
}

void numericRead(numericSource_t *pinSource) {
	// Fonction de lecture d'un port numerique
	// pinSource: Objet indiquant quelle entree lire pour le thread specifique
    uint8_t savedInput = 0;
    uint8_t tempInput = 0;

    while (1) {
		// Lecture initiale de la valeur
        tempInput = pinSource->inputSource.read();

		// Detection d'une variation
        if (tempInput != savedInput) {
			// Utilisation de la fonction verify() pour assurer la stabilite
            if (verify(pinSource->inputSource, tempInput)) {
                savedInput = tempInput;
								
				// Demande au RTC de mettre un timestamp dans la mailbox
				semaphore.release();
				// Boucle d'attente pour une valeur de temps dans la mailbox. C'est pour garantir une vrai valeur
				// si un autre thread "mange" le message dans la mailbox
				while (1) {
					osEvent event = mailbox.get();
					if (event.status == osEventMail) {
						mail_t *mail = (mail_t*) event.value.p;
						if (mail->isValid) {
							// Envoi de l'evenement
							sendEvent(tempInput, mail->timestamp, 1, pinSource->pin);
							mailbox.free(mail);
							break;
						}
						Thread::wait(5);
					}
				}
				Thread::wait(45);
            }
        }
        Thread::wait(5);
		// Attente qui varie entre 100ms et 100ms + (5ms * attente de mailbox) au total pour la lecture numerique
    }
}

float convertAnalog(uint16_t input) {
	// Conversion de la valeur hex en voltage sur 3.3 volt pour meilleure comprehension humaine des valeurs
	// input: Representation du voltage sur un uint16 ou 0x00 = 0v et 0xFFFF = 3.3v
	return ((input * 3.3) / 0xFFFF);
}

void analogRead() {
	// Fonction de lecture des deux ports analogiques
    uint16_t differential[2] = {0};
    uint16_t savedAverage[2] = {0};
    int average[2] = {0};
    uint16_t currentInput[10] = {0};
	bool dateFetched = false;
	struct tm *dateTime = 0;

    while (1) {
        average[0] = 0;
		average[1] = 0;

        // Lectures sur 250ms
        for (uint8_t index = 0; index < 10; index += 2) {
			currentInput[index] = analogInput1.read_u16();
			currentInput[index + 1] = analogInput2.read_u16();
            Thread::wait(50);
        }

        // Calcul de la moyenne
        for (uint8_t index = 0; index < 10; index += 2) {
            average[0] += currentInput[index];
            average[1] += currentInput[index+1];
        }
        average[0] /= 5;
        average[1] /= 5;
		
		// Calcul du 12.5% de la plage maximale
        differential[0] = (savedAverage[0] * 125) / 1000;
        differential[1] = (savedAverage[1] * 125) / 1000;

        for (uint8_t index = 0; index < 2; index++) {
			// Verification si une variation assez importante
            if (average[index] < (savedAverage[index] - differential[index]) || average[index] > (savedAverage[index] + differential[index])) {
				savedAverage[0] = average[0];
				savedAverage[1] = average[1];
				// Verification si on a deja obtenu la date pour les deux lectures
				if (!dateFetched) {
					// Demande au RTC d'avoir une date
					semaphore.release();
					while (1) {
						// Attente de l'obtention d'une date
						osEvent event = mailbox.get();
						if (event.status == osEventMail) {
							mail_t *mail = (mail_t*) event.value.p;
								if (mail->isValid) {
									dateTime = mail->timestamp;
									mailbox.free(mail);
									dateFetched = true;
									break;
								}
							Thread::wait(5);
						}
					}	
				}
				// Envoi de l'evenement
				sendEvent(convertAnalog(average[index]), dateTime, 2, index);
            }
        }
		dateFetched = false;
    }
}

void collect() {
    while (true) {
		// Lecture dans la queue
        osEvent evt = queue.get();
        if (evt.status == osEventMessage) {
            event_t *event = (event_t*)evt.value.p;
			// Verification de l'origine de l'evenement (numerique / analogique et quelle pin)
			if (event->type == 1) {
				printf("Type: Numerique");
				if (event->pin == 0) {
					printf(" (p15)\n\r");
				} else {
					printf(" (p16)\n\r");
				}
			}
			else if (event->type == 2) {
				printf("Type: Analogique");
				if (event->pin == 0) {
					printf(" (p19)\n\r");
				} else {
					printf(" (p20)\n\r");
				}
			}
			// Imprimer la valeur detectee dans l'evenement
            printf("Value: %f \n\r", event->value);
			// Format de la date demande
			char dateBuffer[80] = {'0'};
			strftime(dateBuffer, 80, "%y:%m:%d:%H:%M:%S", event->timestamp);
			printf(dateBuffer);
			printf("\n\n\r");
			mpool.free(event);
        }
    }
}

void rtc() {
	while (1) {
		// Systeme de credit pour l'attente du RTC
		semaphore.wait();
		mail_t *mail = mailbox.alloc();
		mail->isValid = true;
		// Calcul du temps par rapport au set_time() fait dans le main()
		time_t t = time(0);
		mail->timestamp = localtime(&t);
		mailbox.put(mail);
		// Attente de 50ms entre chaque interaction avec RTC
		Thread::wait(50);
    }
}

int main() {
	set_time(1517323168);
    
	// Configucation du parametre pour la lecture numerique
	numericSource_t pin15 = {numericInput1, 0};
	numericSource_t pin16 = {numericInput2, 1};
	
	// Creation des threads
	Thread numericReadThread1;
    Thread numericReadThread2;
    Thread analogReadThread;
    Thread collectionThread;
	Thread rtcThread;

	// Initialisation des threads de lecture numerique
    numericReadThread1.start(callback(numericRead, &pin15));
    numericReadThread2.start(callback(numericRead, &pin16));
    numericReadThread1.set_priority(osPriorityHigh);
    numericReadThread2.set_priority(osPriorityHigh);

	// Initialisation du thread de lecture analogique
    analogReadThread.start(analogRead);
    analogReadThread.set_priority(osPriorityAboveNormal);

	// Initialisation du thread de collection des evenements
    collectionThread.start(collect);
    collectionThread.set_priority(osPriorityNormal);
	
	// Initialisation du thread du RTC
	rtcThread.start(rtc);
	rtcThread.set_priority(osPriorityNormal);

    while (1) {
			Thread::wait(10000);
    }
}
