#include <mbed.h>
#include <rtos.h>

DigitalOut myLed(LED1);
DigitalIn numericInput1(p15);
DigitalIn numericInput2(p16);
AnalogIn analogInput1(p19);
AnalogIn analogInput2(p20);

typedef struct {
    uint16_t value;
    int dateTime;
} event_t;

MemoryPool<event_t, 50> mpool;
Queue<event_t, 50> queue;
Mail<int, 2> timestampMailbox;
Mutex mutex;

void sendEvent(uint16_t value) {
    event_t *event = mpool.alloc();
    event->value = value;
    event->dateTime = 0;

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

                sendEvent(tempInput);

                Thread::wait(45);
            }
        }
        Thread::wait(5);
    }
}

void analogRead() {
    uint16_t differential[2] = {0};
    uint16_t savedAverage[2] = {0};
    int average[2] = {0};
    uint16_t currentInput[10] = {0};

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
                sendEvent(average[index]);
            }
        }
    }
}

void collect() {
    while (true) {
        osEvent evt = queue.get();
        if (evt.status == osEventMessage) {
            event_t *event = (event_t*)evt.value.p;
            printf("Go fuck yourself\n\r");
            printf("Current: %d \n\r"     , event->value);
            
            mpool.free(event);
        }
    }
}

int main() {
    Thread numericReadThread1;
    Thread numericReadThread2;
    Thread analogReadThread;
    Thread collectionThread;

    numericReadThread1.start(callback(numericRead, &numericInput1));
    numericReadThread2.start(callback(numericRead, &numericInput2));
    numericReadThread1.set_priority(osPriorityHigh);
    numericReadThread2.set_priority(osPriorityHigh);

    analogReadThread.start(analogRead);
    analogReadThread.set_priority(osPriorityAboveNormal);

    collectionThread.start(collect);
    collectionThread.set_priority(osPriorityNormal);

    while (1) {
        semaphore.wait();
        timestampMailbox.
    }

}