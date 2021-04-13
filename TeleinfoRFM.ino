  //--------------------------------------------------------------------
//                 +/
//                 `hh-
//        ::        /mm:
//         hy`      -mmd
//         omo      +mmm.  -+`
//         hmy     .dmmm`   od-
//        smmo    .hmmmy    /mh
//      `smmd`   .dmmmd.    ymm
//     `ymmd-   -dmmmm/    omms
//     ymmd.   :mmmmm/    ommd.
//    +mmd.   -mmmmm/    ymmd-
//    hmm:   `dmmmm/    smmd-
//    dmh    +mmmm+    :mmd-
//    omh    hmmms     smm+
//     sm.   dmmm.     smm`
//      /+   ymmd      :mm
//           -mmm       +m:
//            +mm:       -o
//             :dy
//              `+:
//--------------------------------------------------------------------
//   __|              _/           _ )  |
//   _| |  |   ` \    -_)   -_)    _ \  |   -_)  |  |   -_)
//  _| \_,_| _|_|_| \___| \___|   ___/ _| \___| \_,_| \___|
//--------------------------------------------------------------------
//--------------------------------------------------------------------
// 2021/01/15 - FB V1.0.0
//--------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <jled.h>
#define MAX_XXTEA_DATA8 200
#include <xxtea-iot-crypt.h>

#define VERSION   "v1.0.0"

#define GATEWAY_ADDRESS 1
#define CLIENT_LINKY_ADDRESS 2

#define MY_BAUD_RATE 9600    // mode standard

#define POWER_PIN         4
#define TELEINFO_LED_PIN  3   // Teleinfo LED
#define TRANS_LED_PIN     5   // Trans LED

#define ENTETE '$'
#define CRYPT_PASS "FumeeBleue"

//-------------------------------------------------------------------------------------
// Seuil : 4,5v à l'entrée du régulateur 3,3v > 4,5v*100k/(100k+68k)=2,68v sur A0
//                                                 3,3v->1023 => 2,68->830
//-------------------------------------------------------------------------------------
#define SEUIL_CHARGE   820

#define RFM_TX_POWER 20   // 5..23 dBm, 13 dBm is default

#define NB_SEND_INFO   5


#define CHILD_ID_ADSC     0
#define CHILD_ID_VTIC     1
#define CHILD_ID_NGTF     2
#define CHILD_ID_LTARF    3
#define CHILD_ID_EAST     4
#define CHILD_ID_IRMS1    5
#define CHILD_ID_IRMS2    6
#define CHILD_ID_IRMS3    7
#define CHILD_ID_URMS1    8
#define CHILD_ID_URMS2    9
#define CHILD_ID_URMS3    10
#define CHILD_ID_PREF     11
#define CHILD_ID_PCOUP    12
#define CHILD_ID_SINSTS   13
#define CHILD_ID_SINSTS1  14
#define CHILD_ID_SINSTS2  15
#define CHILD_ID_SINSTS3  16
#define CHILD_ID_STGE     17
#define CHILD_ID_MSG1     18
#define CHILD_ID_NTARF    19
#define CHILD_ID_NJOURF   20
#define CHILD_ID_NJOURF1  21
#define CHILD_ID_EAIT     22
#define CHILD_ID_SINSTI   23
#define CHILD_ID_EASF01   24
#define CHILD_ID_EASF02   25
#define CHILD_ID_EASF03   26
#define CHILD_ID_EASD01   34
#define CHILD_ID_EASD02   35
#define CHILD_ID_EASD03   36
#define CHILD_ID_ERQ1     38
#define CHILD_ID_ERQ2     39
#define CHILD_ID_ERQ3     40
#define CHILD_ID_SINSTSmin   60
#define CHILD_ID_SINSTSmax   61
#define CHILD_ID_LINKY    99


// Variables Téléinfo---------------------
struct teleinfo_s {
  char _ADSC[13]="";  // Adresse Compteur
  char VTIC[3]="";    
  char NGTF[17]="";
  char LTARF[17]="";  // Libelle tarif
  unsigned long EAST=0; // Energie active soutiree totale
  unsigned long EAIT=0; // Energie active injectee
  unsigned int IRMS1=0; // Courant efficace, phase 1
  unsigned int IRMS2=0; // Courant efficace, phase 2
  unsigned int IRMS3=0; // Courant efficace, phase 3
  unsigned int URMS1=0; // Tension efficace, phase 1
  unsigned int URMS2=0; // Tension efficace, phase 2
  unsigned int URMS3=0; // Tension efficace, phase 3
  unsigned int PREF=0;
  unsigned int PCOUP=0; // Puissance coupure
  unsigned int SINSTS=0; // Puissance apparente
  unsigned int SINSTSmin=0;
  unsigned int SINSTSmax=0;
  unsigned int SINSTS1=0; // Puissance apparente phase 1
  unsigned int SINSTS2=0; // Puissance apparente phase 2
  unsigned int SINSTS3=0; // Puissance apparente phase 3
  unsigned int SINSTI=0; // Puissance apparente injectee
  char STGE[9]=""; // Registre de Statuts
  char MSG1[33]="";
  char NTARF[3]=""; // Index tarifaire en cours
  char NJOURF[3]=""; //Jour en cours
  char NJOURF1[3]=""; // Prochain jour
  unsigned long EASF01=0;
  unsigned long EASF02=0;
  unsigned long EASF03=0;
  unsigned long EASD01=0;
  unsigned long EASD02=0;
  unsigned long EASD03=0;
  unsigned long ERQ1=0; // Energie reactive Q1 totale
  unsigned long ERQ2=0; // Energie reactive Q2 totale
  unsigned long ERQ3=0; // Energie reactive Q3 totale
  unsigned long ERQ4=0; // Energie reactive Q4 totale
}; 

struct teleinfo_s teleinfo;

unsigned int cpt_send_info = NB_SEND_INFO;
unsigned int step_envoi = 0;
unsigned int step_info = 0;
unsigned int step_mono = 0;
unsigned int step_prod = 0;

boolean mode_producteur = false;
boolean mode_triphase = false;


auto led_chg = JLed(TRANS_LED_PIN);

//-------------------------------------------------------------------- CHANGE
void change_etat_led_teleinfo()
{
  static int led_state;

  led_state = !led_state;
  digitalWrite(TELEINFO_LED_PIN, led_state);

}

// ---------------------- ------------------------------------------ LoRa_sendMessage
void LoRa_sendMessage(String message) {

  LoRa.idle();
  LoRa.beginPacket();    // start packet
  LoRa.print(message);   // add payload
  LoRa.endPacket();      // finish packet and send it
  LoRa.sleep();

}

// Envoyer trame teleinfo ------------------------------------------ envoyer_trame
void envoyer_trame(String etiquette, String valeur)
{
String trame;

  if (valeur.length() > 0) {
    digitalWrite(TRANS_LED_PIN, HIGH);

    trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" +  String(GATEWAY_ADDRESS) + ";" + xxtea.encrypt(etiquette + ";" + valeur + ";");
    
    Serial.print(F("Send char: "));
    Serial.print(trame.length());
    Serial.print(F(": "));
    Serial.println(trame);
    LoRa_sendMessage(trame);
    digitalWrite(TRANS_LED_PIN, LOW);
  }
}

//-------------------------------------------------------------------- send_teleinfo
boolean send_teleinfo_info()
{
boolean rc = false;
String trame, data;

  digitalWrite(TRANS_LED_PIN, HIGH);
  switch (step_info) {

    // deux vagues pour ne pas dépasser les 200 caractères max pour LORA
    case 0 : // info première vague
      if (test_charge_condo() == true) {
        trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
        data = String(CHILD_ID_ADSC) + ";" + teleinfo._ADSC + ";";
        data += String(CHILD_ID_NGTF) + ";" + teleinfo.NGTF + ";";
        data += String(CHILD_ID_LINKY) + ";1;";
        trame += xxtea.encrypt(data);
        Serial.print(F(">> Send info 1 vague "));
        Serial.print(trame.length());
        Serial.print(F(": "));
        Serial.println(trame);
    
        LoRa_sendMessage(trame);
    
        digitalWrite(TRANS_LED_PIN, LOW);
        step_info++;
      }
      else digitalWrite(TRANS_LED_PIN, HIGH);
      break;
      
    case 1 : // info deuxième vague
      if (test_charge_condo() == true) {
        trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
        data += String(CHILD_ID_NJOURF) + ";" + teleinfo.NJOURF + ";";
        data += String(CHILD_ID_NJOURF1) + ";" + teleinfo.NJOURF1 + ";";
        data += String(CHILD_ID_LINKY) + ";2;";
        trame += xxtea.encrypt(data);
        Serial.print(F(">> Send info 2 vague "));
        Serial.print(trame.length());
        Serial.print(F(": "));
        Serial.println(trame);
    
        LoRa_sendMessage(trame);
    
        digitalWrite(TRANS_LED_PIN, LOW);
        step_info=0;
        rc=true;
      }
      else digitalWrite(TRANS_LED_PIN, HIGH);
      break;
  }

  return rc;
}

//-------------------------------------------------------------------- send_teleinfo
boolean send_teleinfo_monophase()
{
boolean rc = false;
String trame, data;

  digitalWrite(TRANS_LED_PIN, HIGH);
  switch (step_mono) {

    // deux vagues pour ne pas dépasser les 200 caractères max pour LORA
    case 0 : // mono première vague
      if (test_charge_condo() == true) {
        trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
        data = String(CHILD_ID_SINSTS) + ";" + teleinfo.SINSTS + ";";
        data += String(CHILD_ID_SINSTSmin) + ";" + teleinfo.SINSTSmin + ";";
        data += String(CHILD_ID_SINSTSmax) + ";" + teleinfo.SINSTSmax + ";";
        data += String(CHILD_ID_LINKY) + ";3;";
        teleinfo.SINSTSmin = teleinfo.SINSTS;
        teleinfo.SINSTSmax = teleinfo.SINSTS;
        trame += xxtea.encrypt(data);
        Serial.print(F(">> Send mono 1 vague "));
        Serial.print(trame.length());
        Serial.print(F(": "));
        Serial.println(trame);
    
        LoRa_sendMessage(trame);
    
        digitalWrite(TRANS_LED_PIN, LOW);
        step_mono++;
      }
      else digitalWrite(TRANS_LED_PIN, HIGH);
      break;

    case 1 : // mono deuxième vague
      if (test_charge_condo() == true) {
        trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
        data += String(CHILD_ID_EAST) + ";" + teleinfo.EAST + ";";
        data += String(CHILD_ID_LTARF) + ";" + teleinfo.LTARF + ";";
        data += String(CHILD_ID_NTARF) + ";" + teleinfo.NTARF + ";";
        data += String(CHILD_ID_LINKY) + ";4;";
        teleinfo.SINSTSmin = teleinfo.SINSTS;
        teleinfo.SINSTSmax = teleinfo.SINSTS;
        trame += xxtea.encrypt(data);
        Serial.print(F(">> Send mono 2 vague "));
        Serial.print(trame.length());
        Serial.print(F(": "));
        Serial.println(trame);
    
        LoRa_sendMessage(trame);
    
        digitalWrite(TRANS_LED_PIN, LOW);
        step_mono++;
      }
      else digitalWrite(TRANS_LED_PIN, HIGH);
      break;

    case 2 : // mono troisième vague
      if (test_charge_condo() == true) {
        trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
        data = String(CHILD_ID_EASF01) + ";" + teleinfo.EASF01 + ";";
        data += String(CHILD_ID_EASF02) + ";" + teleinfo.EASF02 + ";";
        data += String(CHILD_ID_EASF03) + ";" + teleinfo.EASF03 + ";";
        data += String(CHILD_ID_LINKY) + ";5;";
        trame += xxtea.encrypt(data);
        Serial.print(F(">> Send mono 3 vague "));
        Serial.print(trame.length());
        Serial.print(F(": "));
        Serial.println(trame);
    
        LoRa_sendMessage(trame);
    
        digitalWrite(TRANS_LED_PIN, LOW);
        step_mono=0;
        rc=true;
      }
      else digitalWrite(TRANS_LED_PIN, HIGH);
      break;

  }
  return rc;
}

//-------------------------------------------------------------------- send_teleinfo_producteur
boolean send_teleinfo_producteur()
{
boolean rc = false;
String trame, data;

  digitalWrite(TRANS_LED_PIN, HIGH);
  switch (step_mono) {
  // deux vagues pour ne pas dépasser les 200 caractères max pour LORA
    case 0 : // prod première vague
      if (test_charge_condo() == true) {
        trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
        data = String(CHILD_ID_EAIT) + ";" + teleinfo.EAIT + ";";
        data += String(CHILD_ID_ERQ1) + ";" + teleinfo.ERQ1 + ";";
        data += String(CHILD_ID_LINKY) + ";6;";
        teleinfo.SINSTSmin = teleinfo.SINSTS;
        teleinfo.SINSTSmax = teleinfo.SINSTS;
        trame += xxtea.encrypt(data);
        Serial.print(F(">> Send prod 1 vague "));
        Serial.print(trame.length());
        Serial.print(F(": "));
        Serial.println(trame);
    
        LoRa_sendMessage(trame);
    
        digitalWrite(TRANS_LED_PIN, LOW);
        step_prod++;
      }
      else digitalWrite(TRANS_LED_PIN, HIGH);
      break;

    case 2 : // prod deuxime vague
      if (test_charge_condo() == true) {
        trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
        data += String(CHILD_ID_ERQ2) + ";" + teleinfo.ERQ2 + ";";
        data += String(CHILD_ID_ERQ3) + ";" + teleinfo.ERQ3 + ";";
        data += String(CHILD_ID_SINSTI) + ";" + teleinfo.SINSTI + ";";
        data += String(CHILD_ID_LINKY) + ";7;";
        trame += xxtea.encrypt(data);
        Serial.print(F(">> Send prod 2 vague "));
        Serial.print(trame.length());
        Serial.print(F(": "));
        Serial.println(trame);
    
        LoRa_sendMessage(trame);
    
        digitalWrite(TRANS_LED_PIN, LOW);
        step_prod=0;
        rc=true;
      }
      else digitalWrite(TRANS_LED_PIN, HIGH);
      break;
  }

  return rc;
}

//-------------------------------------------------------------------- send_teleinfo_triphase
boolean send_teleinfo_triphase()
{
boolean rc = false;
String trame, data;

  digitalWrite(TRANS_LED_PIN, HIGH);
  if (test_charge_condo() == true) {
    
    trame = String(ENTETE) + String(CLIENT_LINKY_ADDRESS) + ";" + String(GATEWAY_ADDRESS) + ";";
    data += String(CHILD_ID_SINSTS1) + ";" + teleinfo.SINSTS1 + ";";
    data += String(CHILD_ID_SINSTS2) + ";" + teleinfo.SINSTS2 + ";";
    data += String(CHILD_ID_SINSTS3) + ";" + teleinfo.SINSTS3 + ";";
    data += String(CHILD_ID_LINKY) + ";8;";
    trame += xxtea.encrypt(data);
    Serial.print(F(">> Send tri: "));
    Serial.print(trame.length());
    Serial.print(F(": "));
    Serial.println(trame);

    LoRa_sendMessage(trame);

    digitalWrite(TRANS_LED_PIN, LOW);
    rc=true;
  }

  return rc;
}

//-------------------------------------------------------------------- send_boot
void send_boot() 
{
  Serial.println(F(">> Send BOOT"));
  verif_charge_condo();
  envoyer_trame(String(CHILD_ID_LINKY), "0");
}

//-------------------------------------------------------------------- charge_condo
bool charge_condo()
{
boolean flag_cap = false;

  Serial.println(F(">> Charge Condo.."));
  led_chg.Breathe(1000).DelayAfter(500).Forever();

  while (flag_cap == false) {
    if (analogRead(A0) >= SEUIL_CHARGE) flag_cap = true;
    led_chg.Update();
  }
  led_chg.Off();
  digitalWrite(POWER_PIN, HIGH);
}

//-------------------------------------------------------------------- test_charge_condo
boolean test_charge_condo()
{
boolean rc = true;
 
  if (analogRead(A0) < SEUIL_CHARGE) {
    rc=false;
  }
  
  return rc;
}

//-------------------------------------------------------------------- verif_charge_condo
void verif_charge_condo()
{
  if (test_charge_condo() == false) charge_condo();  
}


//-------------------------------------------------------------------- SETUP
void setup()
{
  pinMode(TELEINFO_LED_PIN, OUTPUT);
  pinMode(TRANS_LED_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  
  digitalWrite(POWER_PIN, LOW);

  digitalWrite(TELEINFO_LED_PIN, LOW);
  digitalWrite(TRANS_LED_PIN, LOW);
  led_chg.Off();

  Serial.begin(MY_BAUD_RATE);

  Serial.println(F("   __|              _/           _ )  |"));
  Serial.println(F("   _| |  |   ` \\    -_)   -_)    _ \\  |   -_)  |  |   -_)"));
  Serial.println(F("  _| \\_,_| _|_|_| \\___| \\___|   ___/ _| \\___| \\_,_| \\___|"));
  Serial.print(F("                                             "));
  Serial.println(VERSION);

  xxtea.setKey(CRYPT_PASS);

  verif_charge_condo();
  digitalWrite(POWER_PIN, HIGH);

  Serial.print(F("Init RF95: "));
  while (!LoRa.begin(868E6)) {
    Serial.println(F("Starting LoRa failed!"));
    charge_condo();
  }
  LoRa.enableCrc();
  LoRa.sleep();
  Serial.println(F("OK."));

  send_boot();

}


//-------------------------------------------------------------------- LOOP
void loop()
{
  uint32_t currentTime = millis();
 
  read_teleinfo();

  switch (step_envoi) {

    case 0 : // info
      if (cpt_send_info >= NB_SEND_INFO) {
        if (send_teleinfo_info() == true) {
          step_envoi++;
          cpt_send_info=0;
        }
      } 
      else {
        cpt_send_info++;
        step_envoi++;
      }
      break;
      
    case 1 : // monophasé
      if (send_teleinfo_monophase() == true) step_envoi++;
      break;

    case 2 : // triphasé
      if (mode_triphase) {
        if (send_teleinfo_triphase() == true) step_envoi++;
      }
      else step_envoi++;
      break;

    case 3 : // producteur
      if (mode_producteur) {
        if (send_teleinfo_producteur() == true) step_envoi=0;
      }
      else step_envoi=0;
      break;
  }
 
}
