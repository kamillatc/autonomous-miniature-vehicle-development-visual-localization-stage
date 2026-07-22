#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/int32.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t pul;
std_msgs__msg__Int32 msg_pul;

// ===================== CONFIG =====================
#define ENCODER_A 4
#define ENCODER_B 19
#define PULSOS_POR_VOLTA 1000
#define JANELA_MS 10
#define PUBLISH_MS 10   // 100 Hz de publicação

volatile uint32_t pulsos_A = 0;
volatile uint32_t pulsos_B = 0;
volatile int32_t rpm = 0;
volatile uint32_t pulsos_BS = 0;
volatile uint32_t pulsos_AS = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t freqMutex;

// ===================== INTERRUPÇÕES =====================
void IRAM_ATTR contaPulsoA() {
  portENTER_CRITICAL_ISR(&mux);
  pulsos_A++;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR contaPulsoB() {
  portENTER_CRITICAL_ISR(&mux);
  pulsos_B++;
  portEXIT_CRITICAL_ISR(&mux);
}

// ===================== TASK ENCODER (CORE 1) =====================
void taskEncoder(void *pvParameters) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(JANELA_MS));
    portENTER_CRITICAL(&mux);

      uint32_t leitura = pulsos_A;
      pulsos_AS = pulsos_A;
      pulsos_BS = pulsos_B;
      pulsos_A = 0;
      pulsos_B = 0;
    portEXIT_CRITICAL(&mux);

      uint32_t janelas_por_minuto = 60000 / JANELA_MS; // ex: 6000 para JANELA_MS = 10

      if(pulsos_AS < pulsos_BS){
        rpm = ((leitura * janelas_por_minuto) / PULSOS_POR_VOLTA) / 2;
      }
      else{
        rpm = ((leitura * janelas_por_minuto) / PULSOS_POR_VOLTA) / 2;
      }
//    vTaskDelay(1);
  }
}

// ===================== TASK MICRO-ROS (CORE 0) =====================
void taskMicroROS(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(PUBLISH_MS);
  for(;;) {
    msg_pul.data = rpm;
    rcl_publish(&pul, &msg_pul, NULL);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("ESP32 Serial2 OK");
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_encoder", "", &support);
  rclc_publisher_init_default(
    &pul,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "RPM"
  );

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), contaPulsoA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), contaPulsoB, RISING);

  xTaskCreatePinnedToCore(
    taskEncoder,
    "TaskEncoder",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    taskMicroROS,
    "TaskMicroROS",
    8192,
    NULL,
    1,
    NULL,
    0
  );
}

// ===================== LOOP =====================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
