#include <esp_now.h>
#include <WiFi.h>

// Receiver's MAC address
uint8_t broadcastAddress[] = {0x98, 0xcd, 0xac, 0xf7, 0xce, 0x7c};

int LED0_PIN = 18; 
int LED1_PIN = 19;

int LED_RECEIVED_PIN = 9;

// 00  local and confirmed = LOW
// 10 local HIGH is transmitted, confirmed not yet
// 11 local HIGH is transmitted, confirmed HIGH
// 01 local LOW is transmitted, confirmed not yet

bool LED0_state = LOW; 
bool confirmed_LED0_state = LOW;
unsigned long prev_time_Led0 = 0;


bool LED1_state = LOW;
bool confirmed_LED1_state = LOW;
unsigned long prev_time_Led1 = 0;

bool* LEDs_State[2] = {
  &LED0_state,
  &LED1_state
};

bool* Confirmed_LEDs_State[2]{
  &confirmed_LED0_state,
  &confirmed_LED1_state,
};

unsigned long* Prev_times_leds[2]{
  &prev_time_Led0,
  &prev_time_Led1
};

int LED_pins[2]{
  18,
  19
};


bool problem_state = HIGH; 

int BUTTON0_PIN = 14;
int BUTTON1_PIN = 27;

int LEDBuffer0[2] = {0, 0};
int LEDBuffer1[2] = {0, 0};

int* LEDsBuffer[2] = {LEDBuffer0, LEDBuffer1};

int risingEdge[2] = {1, 0}; 
volatile bool deliveryStatus = false;

// Struct for transmitted and received messages
typedef struct struct_transmission_message {
  int transmitted_state0;
  int transmitted_state1;
} struct_transmission_message;

// Create a struct_message called myData
struct_transmission_message myData;
struct_transmission_message receivedData;  // Struct for received data
esp_now_peer_info_t peerInfo;


bool compareArrays(int a[], int b[], int size) {
  for (int i = 0; i < size; i++) {
    if (a[i] != b[i]) {
      return false;  // Return false if any elements differ
    }
  }
  return true;  // Return true if all elements are equal
}

void shiftArrays(int* arrays[], int numArrays, int arraySize, int inputs[]) {
  // Iterate through each array
  for (int i = 0; i < numArrays; i++) {
    // Shift the elements in the current array to the left, leaving the 0th index open for the new one
    for (int j = arraySize - 1; j > 0; j--) {
      arrays[i][j] = arrays[i][j - 1];
    }
    // Set the new input value at index 0
    arrays[i][0] = inputs[i];
  }
}

void set_LED_state( bool* local_Led_states[],bool* confirmed_led_states[],unsigned long* prev_times_leds[],int led_pins[],int size){
  for (int i = 0; i< size; i++){
    if (*local_Led_states[i] == 0 && *confirmed_led_states[i] == 0){
      digitalWrite(led_pins[i],LOW);
    }
    else if (*local_Led_states[i] == 1 && *confirmed_led_states[i] == 1){
      digitalWrite(led_pins[i],HIGH);
    }
    else {

      if (millis() - *prev_times_leds[i] >= 200) {
        *prev_times_leds[i] = millis(); // Update last toggle time

        // Toggle the LED
        digitalWrite(led_pins[i], !digitalRead(led_pins[i]));
    }
    }


  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  deliveryStatus = (status == ESP_NOW_SEND_SUCCESS);
}


void updateConfirmedStates(struct_transmission_message* receivedData, bool* confirmed_led_states[], int size) {
    if (size >= 2) { // Ensure the array is large enough
        *confirmed_led_states[0] = receivedData->transmitted_state0;
        *confirmed_led_states[1] = receivedData->transmitted_state1;
    }
}

// Callback when data is received
void OnDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len) {
  Serial.println("Data received!");

  // Ensure the incoming data is the correct size for our struct
  if (len == sizeof(struct_transmission_message)) {
    // Copy the received data into the `receivedData` struct
    memcpy(&receivedData, data, len);

    updateConfirmedStates(&receivedData, Confirmed_LEDs_State,2);

    // Debugging: Print received values
    Serial.print("Received State 0: ");
    Serial.println(receivedData.transmitted_state0);
    Serial.print("Received State 1: ");
    Serial.println(receivedData.transmitted_state1);
  } else {
    Serial.println("Received data does not match expected struct size.");
  }
}

void setup() {
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED_RECEIVED_PIN, OUTPUT);

  pinMode(BUTTON0_PIN, INPUT);
  pinMode(BUTTON1_PIN, INPUT);

  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for Send CB to get the status of transmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register for Receive Callback
  esp_now_register_recv_cb(OnDataReceived);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;

  // Add peer if the peer does not already exist
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_del_peer(broadcastAddress);  // Delete if it exists, just in case
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }
}

void loop() {
  // Read button states
  int inputs[2] = {
    digitalRead(BUTTON0_PIN),
    digitalRead(BUTTON1_PIN)
  };

  // Shift the input buffer
  shiftArrays(LEDsBuffer, 2, 2, inputs);

  // Check for rising edge and toggle LED states
  for (int i = 0; i < 2; i++) {
    if (compareArrays(LEDsBuffer[i], risingEdge, 2)) {
      *LEDs_State[i] = !(*LEDs_State[i]);
    }
  }

  // Update the transmitted data
  myData.transmitted_state0 = *LEDs_State[0];
  myData.transmitted_state1 = *LEDs_State[1];

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  digitalWrite(LED_RECEIVED_PIN, problem_state);

  // Handle delivery status
  if (deliveryStatus) {
    problem_state = LOW; 
  } else {
    problem_state = HIGH;
  }
  set_LED_state(LEDs_State,Confirmed_LEDs_State,Prev_times_leds,LED_pins,2);

  // Debug information
  // Serial.print("Data transmitted button state: ");
  // Serial.print(myData.transmitted_state0);
  // Serial.println(myData.transmitted_state1);
  // Serial.println(problem_state);

  delay(100);
}
