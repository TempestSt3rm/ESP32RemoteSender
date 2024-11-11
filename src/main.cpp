#include <esp_now.h>
#include <WiFi.h>

//Default Address
uint8_t broadcastAddress[] = {0x98, 0xcd, 0xac, 0xf7, 0xce, 0x7c};

int LED0_PIN = 21; 
int LED1_PIN = 22; 
int LED2_PIN = 19; 
int LED3_PIN = 23; 
int LED4_PIN = 18; 
int LED_RECEIVED_PIN = 9;

bool LED0_state = LOW; 
bool LED1_state = LOW; 
bool LED2_state = LOW; 
bool LED3_state = LOW; 
bool LED4_state = LOW;
bool problem_state = HIGH; 

bool LED_changed = false;

bool* LEDs_State[5] = {
  &LED0_state,
  &LED1_state,
  &LED2_state,
  &LED3_state,
  &LED4_state
};

int BUTTON0_PIN = 26;
int BUTTON1_PIN = 32;
int BUTTON2_PIN = 33;
int BUTTON3_PIN = 27;
int BUTTON4_PIN = 14;

int LEDBuffer0[2] = {0,0};
int LEDBuffer1[2] = {0,0};
int LEDBuffer2[2] = {0,0};
int LEDBuffer3[2] = {0,0};
int LEDBuffer4[2] = {0,0};

int* LEDsBuffer[5] = {LEDBuffer0,LEDBuffer1,LEDBuffer2,LEDBuffer3,LEDBuffer4};

int risingEdge[2] = {1,0}; 
volatile bool deliveryStatus = false;
// Must match the receiver structure
typedef struct struct_message {
  int transmitted_state0;
  int transmitted_state1;
  int transmitted_state2;
  int transmitted_state3;
  int transmitted_state4;
} struct_message;

// Create a struct_message called myData
struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");

  // if (status == ESP_NOW_SEND_FAIL) {
  //       Serial.println("Debug: Send failed, potential causes could be interference, range, or channel mismatch.");
  // }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  deliveryStatus = (status == ESP_NOW_SEND_SUCCESS);
}


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


void setup() {
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED_RECEIVED_PIN, OUTPUT);

  pinMode(BUTTON0_PIN, INPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(BUTTON4_PIN, INPUT);



  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // WiFi.begin("dummySSID", "dummyPassword", 1);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Add peer if the peer does not already exist
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_del_peer(broadcastAddress);  // Delete if it exists, just in case
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

}
 
void loop() {
  // Set values to send
  // myData.button_state = digitalRead(button_pin);

  int inputs[5] = {
    digitalRead(BUTTON0_PIN),
    digitalRead(BUTTON1_PIN),
    digitalRead(BUTTON2_PIN),
    digitalRead(BUTTON3_PIN),
    digitalRead(BUTTON4_PIN),
  };

  shiftArrays(LEDsBuffer,5,2,inputs);
  for (int i = 0; i<5; i++){
    if(compareArrays(LEDsBuffer[i],risingEdge,2)) {
      *LEDs_State[i] = ! *LEDs_State[i];
      LED_changed = true; 
    }
  };

  myData.transmitted_state0 = *LEDs_State[0];
  myData.transmitted_state1 = *LEDs_State[1];
  myData.transmitted_state2 = *LEDs_State[2];
  myData.transmitted_state3 = *LEDs_State[3];
  myData.transmitted_state4 = *LEDs_State[4];

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  digitalWrite(LED_RECEIVED_PIN,problem_state);

  // if (LED_changed){
    
  // }
  problem_state = HIGH;
  

  // Serial.println(result == ESP_OK ? "The sending protocall is successfull" : "Sending failed");
  if (deliveryStatus) {
    // Serial.println("Packet Received activate shift");
    digitalWrite(LED0_PIN,LED0_state);
    digitalWrite(LED1_PIN,LED1_state);
    digitalWrite(LED2_PIN,LED2_state);
    digitalWrite(LED3_PIN,LED3_state);
    digitalWrite(LED4_PIN,LED4_state);  
    problem_state = LOW; 
    LED_changed = false;
  }
  
  
  

  Serial.print("Data transmitted button state:");
  Serial.print(myData.transmitted_state0);
  Serial.print(myData.transmitted_state1);
  Serial.print(myData.transmitted_state2);
  Serial.print(myData.transmitted_state3);
  Serial.println(myData.transmitted_state4);

  Serial.println(problem_state);

  delay(300);
}