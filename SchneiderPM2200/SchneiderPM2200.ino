#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"


#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

BluetoothSerial SerialBT;

// Global copy of slave
#define NUMSLAVES 10
int SlaveCnt = 0;

#define CHANNEL 3
#define PRINTSCANRESULTS 0

String deviceToken = "";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;
String json = "";
  
ModbusMaster node;
void t1CallgetMeter();

void t2CallsendViaNBIOT();
//TASK
Task t1(10000, TASK_FOREVER, &t1CallgetMeter);

Task t2(30000, TASK_FOREVER, &t2CallsendViaNBIOT);
Scheduler runner;
 
struct Meter
{
  String cA;
  String cB;
  String cC;
  String cN;
  String cG;
  String cAvg;
  String vAB;
  String vBC;
  String vCA;  
  String vAN;
  String vBN;
  String vCN;
  
  String vLLAvg;
  String vLNAvg;
  String freq;
  String apTotal;
};
Meter meter;
signal meta ;

void setup()
{
  
   Serial.begin(115200);
  SerialBT.begin("SchneiderPM2200"); //Bluetooth device name
  SerialBT.println("SchneiderPM2200");
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println(F("Starting... Sintrol Monitor"));
  SerialBT.println(F("Starting... Sintrol Monitor"));

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_Meter, modbus);

  Serial.println();
  Serial.println(F("***********************************"));
 

  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
 
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
   
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);

  String ip1 = AISnb.getDeviceIP();
 
  deviceToken = AISnb.getNCCID();
  delay(4000);
  if(deviceToken.length() <1 )
    ESP.restart();
  
}

void t2CallsendViaNBIOT () {
 
  meta = AISnb.getSignal();  
  
  Serial.print("RSSI:"); Serial.println(meta.rssi);
  

  String json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"cA\":");
  json.concat(meter.cA);
  json.concat(",\"cB\":");
  json.concat(meter.cB);
  json.concat(",\"cC\":");
  json.concat(meter.cC);  
  json.concat(",\"cN\":");
  json.concat(meter.cN);  
  json.concat(",\"cG\":");
  json.concat(meter.cG);  
  json.concat(",\"cAvg\":");
  json.concat(meter.cAvg);  
  json.concat(",\"cAvg\":");
  json.concat(meter.cAvg); 
  json.concat(",\"vAB\":");
  json.concat(meter.vAB); 
  json.concat(",\"vBC\":");
  json.concat(meter.vBC);   
  json.concat(",\"vCA\":");
  json.concat(meter.vCA);     

  json.concat(",\"vLLAvg\":");
  json.concat(meter.vLLAvg);  
   json.concat(",\"vLNAvg\":");
  json.concat(meter.vLNAvg);  
  json.concat(",\"vAN\":");
  json.concat(meter.vAN);  
  json.concat(",\"vBN\":");
  json.concat(meter.vBN); 
  json.concat(",\"vCN\":");
  json.concat(meter.vCN);      
  json.concat(",\"freq\":");
  json.concat(meter.freq); 
  json.concat(",\"apTotal\":");
  json.concat(meter.apTotal); 

      
//  json.concat(",\"rssi\":");
//  json.concat(meta.rssi);
//  json.concat(",\"csq\":");
//  json.concat(meta.csq);
//  json.concat(",\"base\":");
//  json.concat(baseline);
//  json.concat(",\"1\":");
//  json.concat(baselineXmedium);
//  json.concat(",\"2\":");
//  json.concat(baselineXhigh);
//  json.concat(",\"wifi\":");
//  json.concat(RSSIWiFi);

  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);

  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  SerialBT.print("rssi:");
  SerialBT.println(meta.rssi);
}
void t1CallgetMeter() {     // Update read all data
 readMeter();
}


float read_Modbus(uint16_t  REG)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 2);
  
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
  }

  value = data[0];
  value = value << 16;
  value = value + data[1];

  val = HexTofloat(value);
  
  return val;
}

float HexTofloat(uint32_t x)
{
  return (*(float*)&x);
}


void readMeter()
{
  meter.cA = read_Modbus(c_A);
  meter.cB = read_Modbus(c_B);
  meter.cC = read_Modbus(c_C);
  meter.cN = read_Modbus(c_N);  
  meter.cG = read_Modbus(c_G);  
  meter.cAvg = read_Modbus(c_Avg);    

  meter.vAB = read_Modbus(v_A_B);  
  meter.vBC = read_Modbus(v_B_C);  
  meter.vCA = read_Modbus(v_C_A);  
  meter.vLLAvg = read_Modbus(v_LL_Avg); 
  meter.vAN = read_Modbus(v_A_N); 
  meter.vBN = read_Modbus(v_B_N); 
  meter.vCN = read_Modbus(v_C_N); 
  meter.vLNAvg = read_Modbus(v_LN_Avg); 
  meter.apTotal = read_Modbus(ap_Total); 
  meter.freq = read_Modbus(v_Freq);

 
  Serial.print("Current A: ");
  Serial.print(meter.cA);
  Serial.println(" Amp");
  Serial.print("Current B: ");
  Serial.print(meter.cB);
  Serial.println(" Amp");
  Serial.print("Current C: ");
  Serial.print(meter.cC);
  Serial.println(" Amp");
  Serial.print("Current N: ");
  Serial.print(meter.cN);
  Serial.println(" Amp");
  Serial.print("Current G: ");
  Serial.print(meter.cG);
  Serial.println(" Amp");
  Serial.print("Current Avg: ");
  Serial.print(meter.cAvg);
  Serial.println(" Amp");
  
  Serial.print("Voltage A-B: ");
  Serial.print(meter.vAB);
  Serial.println(" Volt");
  Serial.print("Voltage B-C: ");
  Serial.print(meter.vBC);
  Serial.println(" Volt");
  Serial.print("Voltage C-A: ");
  Serial.print(meter.vCA);
  Serial.println(" Volt");
  Serial.print("Voltage L-L Avg: ");
  Serial.print(meter.vLLAvg);
  Serial.println(" Volt");
  Serial.print("Voltage A-N: ");
  Serial.print(meter.vAN );
  Serial.println(" Volt");
  Serial.print("Voltage B-N: ");
  Serial.print(meter.vBN );
  Serial.println(" Volt");
  Serial.print("Voltage C-N: ");
  Serial.print(meter.vCN );
  Serial.println(" Volt");
  



  Serial.print("Volt AVG: ");
  Serial.print(meter.vLNAvg);
  Serial.println(" Volt");
  Serial.print("Frequency: ");
  Serial.print(read_Modbus(v_Freq));
  Serial.println(" Hz");

  Serial.print("Active Power Total: ");
  Serial.print(read_Modbus(ap_Total));
  Serial.println(" Kw");
  Serial.println("");
  Serial.println("");

  
  delay(2000);
}

void loop(){
    runner.execute();

  }
