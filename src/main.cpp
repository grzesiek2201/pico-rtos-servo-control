#include <stdio.h>
#include <iostream>
#include <string>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"


#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>

#include <PCA9685_servo_driver.h>
#include <PCA9685_servo.h>


std::string msg{};

TaskHandle_t gLEDtask = NULL;
TaskHandle_t gReceivetask = NULL;
TaskHandle_t gPrinttask = NULL;
TaskHandle_t gMoveServotask = NULL;
TaskHandle_t gFeedbackServotask = NULL;

// SERVO INITIALIZATION
void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange, 
                int16_t maxRange, int16_t position, uint8_t address, 
                uint64_t TConstDur);

PCA9685_servo_driver myController(i2c0, 0, 1, 0x40);
std::vector<PCA9685_servo> myServo = {PCA9685_servo(&myController, 0, 100, 540),
                                      PCA9685_servo(&myController, 1, 100, 540),
                                      PCA9685_servo(&myController, 2, 100, 540)}; // define a vector of servos, note this could extend to use multiple controller on the i2c bus
uint64_t TNow = 0;
uint64_t TPrevious = 0;
uint64_t TEllapsed = 0;


const int RECEIVE_DELAY = 3;


void HeartbeatLEDTask(void *param)
{
    while(1)
    {
        // Blink LED
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void receiveTask(void *param)
{
    int *tick_delay = (int*)param;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(*tick_delay));
        std::getline(std::cin, msg);
        xTaskNotifyGive(gPrinttask);
        
        size_t last = 0; size_t next = 0;
        next = msg.find(" ", last);
        std::string mode = msg.substr(last, next-last);
        last = next + 1;
        next = msg.find(" ", last);

        if(mode == "e") // for 'encoder'
        {
            int nservo = std::stoi(msg.substr(last, next-last));

            std::string enc{};
            enc += std::to_string(nservo % myServo.size()) + " ";
            enc += (std::to_string(myServo.at(nservo % myServo.size()).getPosition()*3.14/180));
            std::cout << enc << " \r\n";
        }
        else if(mode == "p") // for 'position'
        {   
            int nservo = std::stoi(msg.substr(last, next-last));
            last = next + 1;
            next = msg.find(" ", last);
            double angle_rad = std::stod(msg.substr(last, next-last));
            int angle_deg = angle_rad * 180 / 3.14;

            myServo.at(nservo).setPosition(angle_deg);
        }
    }
}

void servoLoopTask(void *param)
{
    while(1)
    {
        TNow = time_us_64();			// time now in microseconds
        TEllapsed = TNow - TPrevious;	// time, in microseconds, since the last loop
        TPrevious = TNow;				// store this ready for the next loop

        for(auto& servo : myServo)
        {
            servo.loop(TEllapsed);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void servoFeedbackTask(void *param)
{
    uint8_t i{0};
    while(1)
    {
        i = 0;
        for(auto& servo : myServo)
        {
            printf("%d %d \r\n", i++, servo.getPosition());
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main()
{
    // Initialize chosen serial port
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("WiFi init failed");
        return -1;
    }
    
    // Initialize controller and servos
    myController.begin(100000);
    int i{0};
    for(auto& servo : myServo){
        init_servo(servo, MODE_FAST, -90, 90, servo.getMidAngle(), i++, 1000000);
        servo.setAngularVelocity(60);
    }
    sleep_ms(1000);

    // Heartbeat LED task, idle priority
    uint32_t status = xTaskCreate(
        HeartbeatLEDTask,
        "Heartbeat LED",
        128,
        nullptr,
        tskIDLE_PRIORITY+1,
        &gLEDtask
    );

    // Receive task, idle priority
    xTaskCreate(
        receiveTask,
        "receive",
        1024,
        (void*)&RECEIVE_DELAY,
        tskIDLE_PRIORITY+1,
        &gReceivetask
    );

    // Servo loop task, the highest priority
    xTaskCreate(
        servoLoopTask,
        "servo loop",
        1024,
        nullptr,
        tskIDLE_PRIORITY+2,
        &gMoveServotask
    );

    vTaskStartScheduler();

    // Loop forever
    while (true) {
    }
}


void init_servo(PCA9685_servo& servo, uint8_t mode, int16_t minRange, int16_t maxRange, int16_t position, uint8_t address, uint64_t TConstDur)
{
    servo.setRange(minRange, maxRange);
    servo.setMode(mode);
    servo.setPosition(position); // move to mid point
    servo.setAddress(address);
    servo.setTConstantDuration(TConstDur);
}
