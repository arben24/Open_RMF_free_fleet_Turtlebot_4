#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <rmf_door_msgs/msg/door_state.h>
#include <rmf_door_msgs/msg/door_request.h>

#include <AccelStepper.h> //to control stepper motor 
#include <WiFi.h>
#include <PubSubClient.h> //for MQTT
#include <ArduinoJson.h>  //to handle Json messages
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//stepper initialisation

int step_pin = 33;
int dir_pin = 32;
int sleep_pin = 25;
int enable_pin = 26;
int ref_button = 23;
bool ref_button_state = 0;
AccelStepper stepper(1, step_pin, dir_pin);
int number_of_steps_to_open = 600;
int stepsPerSecond = 200;
int current_position = 0;


//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//important values you have to change based on your setup/System
IPAddress agent_ip(192, 168, 0, 101);
const char* mqtt_server = "192.168.0.101";
size_t agent_port = 8889;
//MQTT credentials
const char *mqtt_username = "admin";
const char *mqtt_password = "admin";
char ssid[] = "YOUR-SSID";
char psk[]= "YOUR_WIFI_PASSWORD";
String door_name= "door2";
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//MQTT init
WiFiClient espClient;
PubSubClient client(espClient);
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//button and led settings
int button_pin = 13;
int green_led_pin = 12;
int red_led_pin = 14;
int blue_led_pin = 27;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//button definitions
int mode = 2;   //there are 3 modes: manual door open(0), manual door closed (1), automatic mode (2)
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//micro ros deklaration
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

rmf_door_msgs__msg__DoorRequest door_msg;
rmf_door_msgs__msg__DoorState state_msg;

#define ARRAY_LEN 50

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//initialisation for Json 
JsonDocument DoorState;
JsonDocument DoorRequest;

// Error check section
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
// this function sets the door mode an publishes it directly
void set_door_mode(u_int32_t door_mode){
  //micro ROS publish
  state_msg.current_mode.value = door_mode;
  RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
  //MQTT publish
  DoorState["current_mode"]["value"] = door_mode;
  String door_state_tmp;
  serializeJson(DoorState, door_state_tmp);
  client.publish("/door_states",(char*)door_state_tmp.c_str());
}

//this funtion moves a servo 
//this servo is directly connected to the door
void move_stepper_up(int target_step){  //target_step = number of steps(absolute value), time = time it takes to open the door in seconds 

  digitalWrite(sleep_pin,HIGH);
  delay(10);

  stepper.moveTo(target_step); 

  while(stepper.distanceToGo() != 0){
    stepper.run(); 
  }
 
  digitalWrite(sleep_pin,LOW);

}

int move_stepper_down(){

  digitalWrite(sleep_pin,HIGH);
  delay(10);

  stepper.setSpeed(stepsPerSecond); 
  ref_button_state = 0;
  while(!ref_button_state){
    stepper.runSpeed(); 
  }

current_position = stepper.currentPosition();

  move_stepper_up(current_position - 20);
  ref_button_state = 0;
  digitalWrite(sleep_pin,LOW);

return current_position; 

}

void IRAM_ATTR button() {
    ref_button_state = 1;
}


// function to open the door
void open_door(){
  Serial.println("open door");
  digitalWrite(green_led_pin,HIGH);
  digitalWrite(red_led_pin,HIGH);
  set_door_mode(1); // 1 = moving

  //her you can write a function that close the door
  move_stepper_up(current_position - number_of_steps_to_open);

  delay(2000);
  digitalWrite(red_led_pin,LOW);
  set_door_mode(2); // 2 = opened
}

// function to close the door
void close_door(){
  Serial.println("close door");
  digitalWrite(green_led_pin,HIGH);
  digitalWrite(red_led_pin,HIGH);
  set_door_mode(1); // 1 = moving

  //her you can write a function that close the door
  current_position = move_stepper_down(); 

  delay(2000);
  digitalWrite(green_led_pin,LOW);
  set_door_mode(0); // 0 = closed
}

//micro ROS subscriber Callback
void subscription_callback(const void *msgin) {
  const rmf_door_msgs__msg__DoorRequest *msg = (const rmf_door_msgs__msg__DoorRequest *)msgin;
  Serial.println("Received door state message:");
  Serial.print("  Door time (sec): ");
  Serial.println(msg->request_time.sec);
  Serial.print("  Door time (nanosec): ");
  Serial.println(msg->request_time.nanosec);
  Serial.print("  Door name: ");
  Serial.println(msg->door_name.data);
  Serial.print("  Requested mode: ");
  Serial.println(msg->requested_mode.value);
  Serial.print("  Requester ID: ");
  Serial.println(msg->requester_id.data);

  //check for manual mode
  if((mode == 0)||(mode == 1)){
    return;
  }

  //check if actual door state is the same as the requested
  if(msg -> requested_mode.value == state_msg.current_mode.value){
    return;
  }

  if(door_name == (msg -> door_name.data)){
    switch (msg -> requested_mode.value){
    case (u_int32_t)0:
      close_door();
      break;
    case (u_int32_t)1:
      Serial.println("No valide action. Only 0(close) and 2(open) are valid");
      break;
    case (u_int32_t)2:
      open_door();
      break;
    default:
    Serial.println("No valide action. Only 0(close) and 2(open) are valid");
      break;
    }
  }
  else{
    Serial.println("not that door");
  }
}

//micro ROS Publisher timer Callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  state_msg.door_time.sec = 0;
  state_msg.door_time.sec = 0;
  state_msg.door_name.data = door_name.begin();
  //state_msg.current_mode.value = (u_int32_t)0;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
  }
  //Serial.println("published");
}

//setup WIFI for MQTT connection
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, psk);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//MQTT Callback
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "/door_requests") {
    Serial.print("callback for /door_states ");
    JsonDocument DoorRequest;
    DeserializationError error = deserializeJson(DoorRequest, messageTemp);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    int time_sec = DoorRequest["request_time"]["sec"];
    int time_nanosec = DoorRequest["request_time"]["nanosec"];
    const char * requester_id = DoorRequest["requester_id"];
    const char * door_name = DoorRequest["door_name"];
    int requested_mode = DoorRequest["requested_mode"]["value"];

//check if actual door state is the same as the requested
  if(requested_mode == state_msg.current_mode.value){
    return;
  }

  if(door_name == (door_name)){
    switch (requested_mode){
    case 0:
      close_door();
      break;
    case 1:
      Serial.println("No valide action. Only 0(close) and 2(open) are valid");
      break;
    case 2:
      open_door();
      break;
    default:
    Serial.println("No valide action. Only 0(close) and 2(open) are valid");
      break;
    }
  }
  else{
    Serial.println("not that door");
  }


  }
}

//MQTT reconect
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client",mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("/door_requests");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

volatile unsigned long last_interrupt = 0;
volatile int update = 0;

void IRAM_ATTR button_press() {
    update = 1;
    if (millis()-last_interrupt > 200)
    {
      last_interrupt = millis();
      mode++;
      if (mode == 3)
      {
      mode = 0;
      }
    }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);

  // Configure micro ros wifi transport
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  //button and led setup
  pinMode(button_pin,INPUT_PULLUP);
  pinMode(green_led_pin,OUTPUT);
  pinMode(red_led_pin,OUTPUT);
  pinMode(blue_led_pin,OUTPUT);

  digitalWrite(red_led_pin,HIGH);
  digitalWrite(blue_led_pin,HIGH);
  
  //interrupt
  attachInterrupt(button_pin, button_press, FALLING);

  //MQTT Setup
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Json init
  DoorState["door_time"]["sec"] = 0;
  DoorState["door_time"]["nanosec"] = 0;
  DoorState["door_name"] = door_name;
  DoorState["current_mode"]["value"] = 0;
  

  //stepper init
  pinMode(ref_button,INPUT_PULLUP);
  attachInterrupt(ref_button, button, FALLING);

  stepper.setMaxSpeed(stepsPerSecond);
  stepper.setAcceleration(100); 
  pinMode(sleep_pin,OUTPUT);
  pinMode(enable_pin,OUTPUT);

  digitalWrite(sleep_pin,HIGH);
  digitalWrite(enable_pin,LOW);


  //micro ros init
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "door1_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_door_msgs, msg, DoorState),
    "/door_states"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_door_msgs, msg, DoorRequest),
    "/door_requests"
  ));

  // Create timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  //initialize ros massages
  door_msg.door_name.data = (char*) malloc(ARRAY_LEN * sizeof(char));
	door_msg.door_name.size = ARRAY_LEN;
	door_msg.door_name.capacity = ARRAY_LEN;

  door_msg.requester_id.data = (char*) malloc(ARRAY_LEN * sizeof(char));
	door_msg.requester_id.size = ARRAY_LEN;
	door_msg.requester_id.capacity = ARRAY_LEN;

  state_msg.current_mode.value = (u_int32_t)0;

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &door_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  ref_button_state = 0;

  current_position = move_stepper_down();

  Serial.println(current_position);

  Serial.println("Setup complete");
}
long lastMsg = 0;
void loop() {
  //MQTT spin
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //MQTT publish every 1 second
  long now = millis();
  if(client.connected() && (now - lastMsg > 1000)){
  String door_state_tmp;
  serializeJson(DoorState, door_state_tmp);
  client.publish("/door_states",(char*)door_state_tmp.c_str());
  lastMsg = now;
  }

  if(update){
    switch (mode)
  {
  case 0:
    digitalWrite(blue_led_pin,LOW);
    open_door();
    break;
  case 1:
    digitalWrite(blue_led_pin,LOW);
    close_door();
    break;
  case 2:
    digitalWrite(blue_led_pin,HIGH);
    break;  
  default:
    digitalWrite(blue_led_pin,HIGH);
    digitalWrite(red_led_pin,HIGH);
    digitalWrite(green_led_pin,HIGH);
    break;
  }
  update = 0;
}

  delay(100);
  //Micro ROS spin
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}