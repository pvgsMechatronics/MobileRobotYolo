#include "MobileRobot.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>

const char* wifi_ssid = "Neve"; 
const char* wifi_password = "soeusei123";
float K[4] = {169.3930, -169.3923, -153.8437, 153.8444}; 
float k[4] = {2, 2};
float d = 0.02;
float sat = 6;
float T = 0.01;
int N=10;
int pin[4] = {26, 27, 32, 33};

WebServer server(80);
MobileRobot cyberpunk(K, k, sat, T, d);

void setup(void) 
{
  Serial.begin(115200);
  IPAddress staticIP(192, 168, 2, 200);
  IPAddress gateway(192, 168, 2, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  Serial.println("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nConnected to ");
  Serial.println(wifi_ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")){
    Serial.println("MDNS responder started!");
  }
  
  server.on("/", [](){
    server.send(200, "text/plain", "online");
  });

  server.on("/log", [](){
    if (cyberpunk.get_on_path()){
      server.send(200, "text/plain", "Robot busy!");
    }
    else{
      server.send(200, "text/plain", cyberpunk.get_log());
    }
  });

  server.on("/path", [](){
    if (cyberpunk.get_on_path()){
      server.send(200, "text/plain", "Robot busy!");
    }
    else if (server.args() > 1){
      server.send(200, "text/plain", "Too many args!");
    }
    else{
      server.send(200, "text/plain", "OK");
      String msg = String(server.arg(0));
      cyberpunk.receive_path(msg); 
    }
  });

  server.on("/calibrate", [](){  
    String msg = String(server.arg(0));
    if (cyberpunk.get_on_path()){
      server.send(200, "text/plain", "Robot busy!");
    }
    else if (server.args() > 1){
      server.send(200, "text/plain", "Too many args!");
    }
    else if (msg.indexOf("[") == -1){
      server.send(200, "text/plain", "Robot pose not send!");
    }
    else{
      server.send(200, "text/plain", "OK");
      //float x, y;
      //x = String(msg.substring(msg.indexOf("[") + 1, msg.indexOf(","))).toFloat();
      //y = String(msg.substring(msg.indexOf(",") + 1, msg.indexOf("]"))).toFloat();
      cyberpunk.calibrate(N, pin[0], pin[1], pin[2], pin[3]);
    }    
  });

  
  server.begin();
  Serial.println("HTTP server started!");
}

void loop(void) 
{
  //server.handleClient(); 
  cyberpunk.calibrate(0.5, 26, 27, 32, 33);
  cyberpunk.follow_path(0.5); //follow paths with 0.5 m/s on reference speed
}


