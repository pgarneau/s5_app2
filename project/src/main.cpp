#include <mbed.h>
#include <rtos.h>

DigitalOut myLed(LED1);
DigitalIn numericInput1(p15);
AnalogIn analogInput1(p19);

typedef struct {
    uint16_t value;
    int dateTime;
} event_t;

MemoryPool<event_t, 50> mpool;
Queue<event_t, 50> queue;
Mutex mutex;

void numericRead() {
    uint8_t savedInput = 1;
    uint8_t currentInput = 0;
    uint8_t tempInput = 0;

    while (1) {
        mutex.lock();
        tempInput = numericInput1;

        if (tempInput != savedInput) {
            Thread::wait(50);
            currentInput = numericInput1;

            if (currentInput == tempInput) {
                savedInput = currentInput;

                event_t *event = mpool.alloc();
                event->value = currentInput;
                event->dateTime = 0;
                queue.put(event);

                mutex.unlock();
            }
            Thread::wait(50);
        } else {
            mutex.unlock();
            Thread::wait(100);
        }
    }
}

void analogRead() {
    float differential = 0.0;
    uint16_t savedAverage = 0;
    uint32_t average = 0;
    uint16_t currentInput[5] = {0};

    while (1) {
        mutex.lock();

        average = 0;

        // Lectures sur 250ms
        for (uint8_t index = 0; index < 5; index++) {
            currentInput[index] = analogInput1.read_u16();
            Thread::wait(50);
        }

        // Calcul de la moyenne
        for (uint8_t index = 0; index < 5; index++) {
            average += currentInput[index];
        }
        average /= 5;

        differential = savedAverage * 0.125;


    }
}

int main() {
    Thread numericReadThread;
    Thread analogReadThread;

    numericReadThread.start(numericRead);
    numericReadThread.set_priority(osPriorityAboveNormal);

    analogReadThread.start(analogRead);
    analogReadThread.set_priority(osPriorityNormal);

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