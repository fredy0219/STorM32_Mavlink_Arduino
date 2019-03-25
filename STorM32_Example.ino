#include "mavlink/include/mavlink_types.h"
#include "mavlink/include/mavlink.h"
#include <SoftwareSerial.h>

// Easy for reading message.
SoftwareSerial mySerial(10, 11); // RX, TX 

#define ToDeg(x) (x*57.2957795131)  // *180/pi

uint8_t SYSTEM_ID = 255; // 0x47
uint8_t COMPONENT_ID = 1; // 0x43
uint8_t TARGET_SYSTEM_ID = 71; // 0x47
uint8_t TARGET_COMPONENT_ID = 67 // 0x43

union byteToFloat
{
    struct
    {
      byte b0 :8;
      byte b1 :8;
      byte b2 :8;
      byte b3 :8;
    } bytes;
    float f;
};

union byteToInt
{
    struct
    {
      byte b0 :8;
      byte b1 :8;
    } bytes;

    uint16_t i;
};

union intFloat
{
    int i;
    float f;
};

void setup(){
  Serial.begin(115200);
  mySerial.begin(115200);

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  //Testing Mavlink API functionality.

  requestAttitude(); // failure - msg.id returen 0
  requestParameter();
  
  recenter(); // ok
  setAngles(); // ok
  
  setRcPitch(1000); // failure
  setRcRoll(1000); // failure
  setRcYaw(1000); // failure

}

uint8_t yaw = 0;
uint8_t inverse = 1;

void loop(){   

//  yaw += inverse;
//  
//  if(yaw < 0){
//    inverse = 1;
//  }
//  if(yaw > 90){
//    inverse = -1;
//  }
//  
//  setAngles(0,0,yaw);
//
//  delay(200);


//  
  read_mavlink_storm32();
//  setAngles(0,0,0)
}


void read_mavlink_storm32(){ 
  
  mavlink_message_t msg;
  mavlink_status_t status;

  
  
  while (mySerial.available() > 0) {


    uint8_t c = mySerial.read();
    
    //trying to grab msg
    uint8_t x = mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status);
    if (x) {   
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_ATTITUDE:
          {
            //get pitch and yaw angle from storm (requestAttitude() must be executed first)
            double gimbalYaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
            double gimbalPitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
          }
          break;
          
        case MAVLINK_MSG_ID_PARAM_VALUE:
          {
            //get parameter value from storm (parameter 66 is pan mode, requestParameter(int id) must be executed first)
            if(mavlink_msg_param_value_get_param_index(&msg) == 66)
              int panMode = mavlink_msg_param_value_get_param_value(&msg);
          }
          break;
        default:
          break;
      }
    }  
  }
  
}

void requestAttitude(){

  // MAVLINK_MSG_ID_COMMAND_LONG
  //  - MAV_CMD_GET_ATTITUDE (#1234, 0x04D2)

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 1234, 0, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

//  Serial.write(buf,len);
  mySerial.write(buf, len); 
 
}

void requestParameter(int id){
     
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_request_read_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, "", id);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//  Serial.write(buf, len);   
  mySerial.write(buf, len);
}

void setParameter(int id, int val){

    // MAVLINK_MSG_ID_COMMAND_LONG
    //  - MAV_CMD_DO_SET_PARAMETER (#180, 0xB4)
    //  -- * param1 = parameter index
    //     * param2 = parameter value

    intFloat parameterValue;
    parameterValue.i = val;
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 180, 0, id, parameterValue.f, 0.0, 0.0, 0.0, 0.0, 0.0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//    Serial.write(buf, len); 
    mySerial.write(buf, len);
      
}


void setAngles(float roll, float pitch, float yaw){

  // MAVLINK_MSG_ID_COMMAND_LONG
  // - MAV_CMD_DO_MOUNT_CONTROL (#205, 0xCD)
  // -- * param1 = pitch, angle in degree, unlimited
  //    * param2 = roll, angle in degree, unlimited
  //    * param3 = yaw, angle in degree, unlimited
  //    * param7 = mount_mode (0 = MAV_MOUNT_MODE_RETRACT and 1 = MAV_MOUNT_MODE_NEUTRAL recenters the camera)
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID , &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 205, 0, pitch, roll, yaw, 0.0, 0.0, 0.0, 0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//  Serial.write(buf, len);
  mySerial.write(buf, len);
  
}

void recenter(){

  // MAVLINK_MSG_ID_COMMAND_LONG 
  //  - MAV_CMD_DO_MOUNT_CONFIGURE (#204, 0xCC)
  //  -- param1 = mount_mode (0 = MAV_MOUNT_MODE_RETRACT and 1 = MAV_MOUNT_MODE_NEUTRAL recenters the camera)
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg ,TARGET_SYSTEM_ID ,TARGET_COMPONENT_ID, 204, 0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//  Serial.write(buf, len);
  mySerial.write(buf, len);
}

void setRcPitch(uint16_t val){
  setRc(0x0A, val);
}

void setRcRoll(uint16_t val){
  setRc(0x0B, val);
}

void setRcYaw(uint16_t val){
  setRc(0x0C, val);
}


void setRc(byte type, uint16_t val){

  // MAVLINK_MSG_ID_COMMAND_LONG
  //  - MAV_CMD_TARGET_SPECIFIC
  //  -- CMD_SETPITCH, CMD_SETROLL, CMD_SETYAW
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  byteToInt tmpVal;
  tmpVal.i = val; 
  
  byteToFloat data1;
  data1.bytes.b0 = 0xFA;
  data1.bytes.b1 = 0x02;
  data1.bytes.b2 = 0x0A;
  data1.bytes.b3 = tmpVal.bytes.b0;

  byteToFloat data2;
  data2.bytes.b0 = tmpVal.bytes.b1;
  data2.bytes.b1 = 0;
  data2.bytes.b2 = 0;
  data2.bytes.b3 = 0;


  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 1235, 0, data1.f, data2.f, 0.0, 0.0, 0.0, 0.0, 0.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  mySerial.write(buf, len);

  //Checking ACK correction.
  for(uint8_t i = 0 ; i < MAVLINK_MAX_PACKET_LEN ; i++)
    Serial.println(buf[i]);
    
  Serial.println();
}


