#include <mbed.h>
#include <rtos.h>
#include <time.h>

DigitalOut myLed(LED1);
DigitalIn numericInput1(p15);
DigitalIn numericInput2(p16);
AnalogIn analogInput1(p19);
AnalogIn analogInput2(p20);

typedef struct {
    uint16_t value;
		int type;
    struct tm *timestamp;
} event_t;

typedef struct {
	struct tm *timestamp;
	int destination;
} mail_t;
	

MemoryPool<event_t, 50> mpool;
Queue<event_t, 50> queue;
Mail<mail_t, 1> mailbox;
Semaphore semaphore;

void sendEvent(uint16_t value, struct tm *date, int type) {
    event_t *event = mpool.alloc();
    event->value = value;
    event->timestamp = date;
		event->type = type;

    queue.put(event);
}

bool verify(DigitalIn *inputSource, uint8_t value) {
    Thread::wait(50);
    if (inputSource->read() == value) {
        return true;
    }
    
    return false;
}

void numericRead(DigitalIn *inputSource) {
    uint8_t savedInput = 1;
    uint8_t tempInput = 0;

    while (1) {
        tempInput = inputSource->read();

        if (tempInput != savedInput) {
            if (verify(inputSource, tempInput)) {
                savedInput = tempInput;
								
								semaphore.release();
								while (1) {
									osEvent event = mailbox.get();
									if (event.status == osEventMail) {
										mail_t *mail = (mail_t*) event.value.p;
										if (mail->destination == 1) {
											sendEvent(tempInput, mail->timestamp, 1);
											mailbox.free(mail);
											break;
										}
										Thread::wait(10);
									}
								}
                Thread::wait(45);
            }
        }
        Thread::wait(5);
    }
}

void analogRead() {
    uint16_t differential[2] = {0};
    uint16_t savedAverage[2] = {0xFFFF, 0xFFFF};
    int average[2] = {0};
    uint16_t currentInput[10] = {0};
		bool dateFetched = false;
		struct tm *dateTime = 0;

    while (1) {
        memset(average, 2, 0);

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

        differential[0] = (savedAverage[0] * 1000) / 125;
        differential[1] = (savedAverage[1] * 1000) / 125;

        for (uint8_t index = 0; index < 2; index++) {
            if (average[index] <= savedAverage[index] - differential[index] || average[index] >= savedAverage[index] + differential[index]) {
                if (!dateFetched) {
									semaphore.release();
									while (1) {
										osEvent event = mailbox.get();
										if (event.status == osEventMail) {
											mail_t *mail = (mail_t*) event.value.p;
											if (mail->destination == 1) {
												dateTime = mail->timestamp;
												mailbox.free(mail);
												dateFetched = true;
												break;
											}
										Thread::wait(10);
										}
									}	
								}
								sendEvent(average[index], dateTime, 2);
            }
        }
				dateFetched = false;
    }
}

void collect() {
    while (true) {
        osEvent evt = queue.get();
        if (evt.status == osEventMessage) {
            event_t *event = (event_t*)evt.value.p;
						if (event->type == 1) {
							printf("Type: Numerique\n\r");
						}
						else if (event->type == 2) {
							printf("Type: Analogique\n\r");
						}
            printf("Value: %d \n\r", event->value);
						printf("Seconds: %d\n\r", event->timestamp->tm_sec);
						//printf("%d:%d:%d:%d:%d:%d\n\r", event->timestamp->tm_year, event->timestamp->tm_mon+1, event->timestamp->tm_mday, event->timestamp->tm_hour, event->timestamp->tm_min, event->timestamp->tm_sec);
            mpool.free(event);
        }
    }
}

void rtc() {
		time_t t;
		time(&t);
	
		while (1) {
			semaphore.wait();
			mail_t *mail = mailbox.alloc();
			mail->destination = 1;
			mail->timestamp = localtime(&t);
      mailbox.put(mail);
			Thread::wait(50);
    }
}

int main() {
    Thread numericReadThread1;
    Thread numericReadThread2;
    Thread analogReadThread;
    Thread collectionThread;
		Thread rtcThread;

    numericReadThread1.start(callback(numericRead, &numericInput1));
    //numericReadThread2.start(callback(numericRead, &numericInput2));
    numericReadThread1.set_priority(osPriorityHigh);
    //numericReadThread2.set_priority(osPriorityHigh);

    //analogReadThread.start(analogRead);
    //analogReadThread.set_priority(osPriorityAboveNormal);

    collectionThread.start(collect);
    collectionThread.set_priority(osPriorityNormal);
	
		rtcThread.start(rtc);
		rtcThread.set_priority(osPriorityNormal);

    while (1) {
			Thread::wait(5000);
    }
}
