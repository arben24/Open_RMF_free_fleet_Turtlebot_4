#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmf_lift_msgs/msg/lift_state.h>
#include <rmf_lift_msgs/msg/lift_request.h>

#include <ESP32Servo.h>   //to controll Servos
#include <WiFi.h>
#include <PubSubClient.h> //for MQTT
#include <ArduinoJson.h>  //to handle Json messages
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//servo initialisation
Servo door;
int servo_pwm_pin = 15;
int actual_angle = 40;
int angle_open = 140;   //value between 0-180 in degrees
int angle_close = 40;   //value between 0-180 in degrees
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//important values you have to change based on your setup/System
IPAddress agent_ip(192, 168, 0, 101);
const char* mqtt_server = "192.168.0.101";
size_t agent_port = 8890;
//MQTT credentials
const char *mqtt_username = "admin";
const char *mqtt_password = "admin";
char ssid[] = "Leobots";
char psk[]= "leobots42";
String lift_name = "lift1";
int num_available_floors = 2;
char * available_floors[2] = {"L1", "L2"};
char * starting_floor = available_floors[0];
char  * session_id = "";
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//MQTT init
WiFiClient espClient;
PubSubClient client(espClient);
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//button and led settings
int button_pin = 26;
int green_led_pin = 25;
int red_led_pin = 33;
int blue_led_pin = 32;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//button definitions
int mode = 2;   //there are 3 modes: manual door open(0), manual door closed (1), automatic mode (2)
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
//micro ros deklaration
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

rmf_lift_msgs__msg__LiftRequest lift_msg;
rmf_lift_msgs__msg__LiftState state_msg;

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
void set_door_mode(u_int8_t door_mode){
  //micro ROS publish
  state_msg.door_state = door_mode;
  RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
  //MQTT publish
  DoorState["current_mode"]["value"] = door_mode;
  String door_state_tmp;
  serializeJson(DoorState, door_state_tmp);
  client.publish("/door_states",(char*)door_state_tmp.c_str());
}

void set_motion_state(uint8_t motion_state){
  state_msg.motion_state = motion_state;
  RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
}

void set_current_floor(int floor){
  state_msg.current_floor.data = available_floors[floor];
  RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
}

void set_current_floor(char* floor){
  state_msg.current_floor.data = floor;
  RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
}

void set_destination_floor(int floor){
  state_msg.destination_floor.data = available_floors[floor];
  RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
}

//this funtion moves a servo 
//this servo is directly connected to the door
void move_servo(int target_angle, float time){  //target_angle = value between 0 and 180, time = time it takes to open the door in seconds 

  int time_per_step = (time * 1000)/(abs(target_angle-actual_angle));
  if(target_angle > actual_angle){
    for (int i = actual_angle; i <= target_angle; i++){
      door.write(i);
      delay(time_per_step);
    }
  }

  else{
    for (int i = actual_angle; i > target_angle; i--){
      door.write(i);
      delay(time_per_step);
    }
  }
  actual_angle = target_angle;
}

// function to open the door
void open_door(){
  Serial.println("open door");
  digitalWrite(green_led_pin,HIGH);
  digitalWrite(red_led_pin,HIGH);
  set_door_mode(1); // 1 = moving

  //her you can write a function that close the door
  move_servo(angle_open,5);

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
  move_servo(angle_close,5 ); 

  delay(2000);
  digitalWrite(green_led_pin,LOW);
  set_door_mode(0); // 0 = closed
}

void move_lift(int floor_num){

  int current_floor;
  set_destination_floor(floor_num);

  for (int i = 0; i < num_available_floors; i++){
    if(state_msg.current_floor.data == available_floors[i]){
      current_floor = i;
    } 
  } 


  if (current_floor < floor_num){
    Serial.println("lift move up");
    for (int i = current_floor; i < floor_num; i++){
      set_current_floor(i);
      set_motion_state(1);
      delay(10000);
      current_floor++;
    }
    set_current_floor(current_floor);
    set_motion_state(0);

  }

  else{
    Serial.println("lift move down");

    for (int i = current_floor; i > floor_num; i--){
      set_current_floor(i);
      set_motion_state(2);
      delay(10000);
      current_floor--;
    }
    set_current_floor(current_floor);
    set_motion_state(0);

  }
}


//micro ROS subscriber Callback
void subscription_callback(const void *msgin) {
  const rmf_lift_msgs__msg__LiftRequest *msg = (const rmf_lift_msgs__msg__LiftRequest *)msgin;
  /*Serial.println("Received lift request message:");
  Serial.print("  Lift time (sec): ");
  Serial.println(msg->request_time.sec);
  Serial.print("  Lift time (nanosec): ");
  Serial.println(msg->request_time.nanosec);
  Serial.print("  Lift name: ");
  Serial.println(msg->lift_name.data);
  Serial.print("  Requested floor: ");
  Serial.println(msg->destination_floor.data);
  Serial.print("  Requester ID: ");
  Serial.println(msg->session_id.data);
  */

  //check for manual mode
  if((mode == 0)||(mode == 1)){
    return;
  }

  //update session id
  state_msg.session_id.data = msg->session_id.data;

  //check if actual door state is the same as the requested
  if(((String)msg -> destination_floor.data == (String)state_msg.current_floor.data) && (msg->door_state == state_msg.door_state)){
    Serial.println("same request");
    return;
  }
  //reset session
  if(msg->request_type == 0){
    state_msg.session_id.data = "";
  }

  if(lift_name == (msg -> lift_name.data)){


  if(!(msg->door_state == state_msg.door_state)){
    if (msg->door_state == 0)
    {
      close_door();
    }

    if (msg->door_state == 2)
    {
      open_door();
    }
  }

    

  if(!((String)msg -> destination_floor.data == (String)state_msg.current_floor.data)){
    for (int i = 0; i < num_available_floors; i++)
    {
      if((String)msg->destination_floor.data == available_floors[i]){
        Serial.println("move lift");
        move_lift(i);
      } 
    }
  }


    


  }

  else{
    Serial.println("not that door");
  }
}

//micro ROS Publisher timer Callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  state_msg.lift_time.sec = 0;
  state_msg.lift_time.sec = 0;

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
  if(requested_mode == 5){
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
    //Serial.println("not that door");
  }


  }
}

//MQTT reconnect
void reconnect() {
  // Loop until succesfully reconnected
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
  DoorState["door_name"] = lift_name;
  DoorState["current_mode"]["value"] = 0;
  

  //servo init
  door.attach(servo_pwm_pin);
  door.write(angle_close);

  //micro ros init
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "lift1_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_lift_msgs, msg, LiftState),
    "/lift_states"));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(rmf_lift_msgs, msg, LiftRequest),
    "/lift_requests"
  ));

  // Create timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));


  Serial.println("start init");
  //initialize ros massages
  state_msg.lift_name.data = (char*) malloc(ARRAY_LEN * sizeof(char));
	state_msg.lift_name.size = ARRAY_LEN;
  state_msg.lift_name.capacity = ARRAY_LEN;
  state_msg.lift_name.data = lift_name.begin();


  state_msg.available_floors.data = (rosidl_runtime_c__String *)malloc(2 * sizeof(rosidl_runtime_c__String));
  state_msg.available_floors.capacity = 2;
  state_msg.available_floors.size = 2;

  for(int i = 0;i < num_available_floors; i++){
    state_msg.available_floors.data[i].data = (char*) malloc((strlen(available_floors[i]) + 1) * sizeof(char));
    state_msg.available_floors.data[i].size = strlen(available_floors[i]) + 1;
    state_msg.available_floors.data[i].capacity = strlen(available_floors[i]) + 1;
    state_msg.available_floors.data[i].data = available_floors[i];

  }

  state_msg.current_floor.data = (char*) malloc(ARRAY_LEN * sizeof(char));
  state_msg.current_floor.size = ARRAY_LEN;
  state_msg.current_floor.capacity = ARRAY_LEN;
  state_msg.current_floor.data = starting_floor;

  state_msg.destination_floor.data = (char*) malloc(ARRAY_LEN * sizeof(char));
  state_msg.destination_floor.size = ARRAY_LEN;
  state_msg.destination_floor.capacity = ARRAY_LEN;
  state_msg.destination_floor.data = starting_floor;

    state_msg.door_state = 0;
  state_msg.motion_state = 0;


state_msg.available_modes.data = (uint8_t *)malloc(6 * sizeof(uint8_t));
state_msg.available_modes.capacity = 6;
state_msg.available_modes.size = 6;

  for(int i = 0;i < 6; i++){

  state_msg.available_modes.data[i] = i;

  }
  state_msg.current_mode = (u_int8_t)2;

  state_msg.session_id.data = (char*) malloc(ARRAY_LEN * sizeof(char));
  state_msg.session_id.size = ARRAY_LEN;
  state_msg.session_id.capacity = ARRAY_LEN;

  state_msg.session_id.data = session_id;

  lift_msg.lift_name.data = (char*) malloc(ARRAY_LEN * sizeof(char));
  lift_msg.lift_name.size = ARRAY_LEN;
  lift_msg.lift_name.capacity = ARRAY_LEN;

  lift_msg.destination_floor.data = (char*) malloc(ARRAY_LEN * sizeof(char));
  lift_msg.destination_floor.size = ARRAY_LEN;
  lift_msg.destination_floor.capacity = ARRAY_LEN;

  lift_msg.session_id.data = (char*) malloc(ARRAY_LEN * sizeof(char));
  lift_msg.session_id.size = ARRAY_LEN;
  lift_msg.session_id.capacity = ARRAY_LEN;

  Serial.println("init finished");

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &lift_msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

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