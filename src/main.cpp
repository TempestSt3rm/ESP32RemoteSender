#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x30, 0x83, 0x98, 0xee, 0x4e, 0x38};
int button_pin = 14;
int component1State_pin = 2;
volatile bool deliveryStatus = false;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int button_state;
} struct_message;

// Create a struct_message called myData
struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  deliveryStatus = status;
}
 
void setup() {
  pinMode(button_pin,INPUT);
  pinMode(component1State_pin,OUTPUT);

  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}
 
void loop() {
  // Set values to send
  myData.button_state = digitalRead(button_pin);
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  // Serial.println(result == ESP_OK ? "The sending protocall is successfull" : "Sending failed");
  if (deliveryStatus == ESP_NOW_SEND_SUCCESS) {
    Serial.println("packet received");
    digitalWrite(component1State_pin,HIGH);
  } else {
    Serial.println("packet not received");
    digitalWrite(component1State_pin,LOW);
  }

  Serial.print("Data transmitted button state:");
  Serial.println(myData.button_state);

  delay(100);
}