/* Tesla Coil Winder Firmware    */
/* 29 Jan 2014 - Jeff Rosowski   */

#include <Wire.h>
#include <LiquidCrystal.h>
#include <avr/eeprom.h>
#include <MenuSystem.h>
#include <stdlib.h>

#define BAUDRATE 19200

//                  "                    "
#define MENU_TITLE  "Telsa Winder        "
#define MANUAL      "Manual Control     "
#define ENABLE      "Stepper Enable     "
#define JOG_COIL    "Jog coil           "
#define JOG_GUIDE   "Jog guide          "
#define SETTINGS    "Settings           "
#define COIL_STEPS  "Coil steps/rot     "
#define COIL_INV    "Coil invert        "
#define GUIDE_STEPS "Guide steps/mm     "
#define GUIDE_INV   "Guide invert       "
#define HOME        "Home guide         "
#define SETUP_JOB   "Setup Job          "
#define SETUP_HELP  "Size in micrometers"
#define WIRE_SIZE   "Wire size          "
#define COIL_DIAM   "Coil diam          "
#define COIL_HOME   "Home offset        "
#define COIL_CALC   "Calc coil size     "
#define COIL_RATIO  "Coil ratio         "
#define COIL_LEN    "Coil len           "
#define COIL_TURNS  "Coil turns         "
#define COIL_SPEED  "Speed mm/sec       "
#define START       "Start              "

MenuSystem ms;
Menu mm(MENU_TITLE);
Menu manual(MANUAL);
MenuItem manual_stepper(ENABLE);
MenuItem manual_jogcoil(JOG_COIL);
MenuItem manual_jogguide(JOG_GUIDE);
Menu settings(SETTINGS);
MenuItem settings_coilsteps(COIL_STEPS);
MenuItem settings_coilinvert(COIL_INV);
MenuItem settings_guidesteps(GUIDE_STEPS);
MenuItem settings_guideinvert(GUIDE_INV);
MenuItem settings_coilhome(COIL_HOME);
MenuItem settings_home(HOME);
Menu job(SETUP_JOB);
MenuItem job_help(SETUP_HELP);
MenuItem job_wiredia(WIRE_SIZE);
MenuItem job_coildia(COIL_DIAM);
MenuItem job_coilhome(COIL_HOME);
MenuItem job_coilcalc(COIL_CALC);
MenuItem job_coilratio(COIL_RATIO);
MenuItem job_length(COIL_LEN);
MenuItem job_turns(COIL_TURNS);
MenuItem job_speed(COIL_SPEED);
MenuItem job_home(HOME);
MenuItem job_start(START);

const int min_stop = 17;
const int max_stop = 16;

const int led = 13;
byte line = 0;
byte cursor_line = 0;

// stepper A
const int stepper_a_enable = 7;
const int stepper_a_direction = 8;
const int stepper_a_step = 9;
long stepper_a_steps = 0;
byte stepper_a_run = 0;
byte stepper_a_dir = 0;
byte stepper_a_ena = 0;

// stepper B
const int stepper_b_enable = 12;
const int stepper_b_direction = 11;
const int stepper_b_step = 10;
long stepper_b_steps = 0;
byte stepper_b_run = 0;
byte stepper_b_dir = 0;
byte stepper_b_ena = 0;

// input buttons
const int button_a = 2;  // ENTER
const int button_b = 3;  // DOWN
const int button_c = 4;  // UP
const int button_d = 5;  // LEFT
const int button_e = 6;  // RIGHT
const int invert_buttons = 1;    // invert buttons

// switch debounce
#define DEBOUNCE_TIME 100
#define LONG_TIME 32000
long a_wait_timer;
long b_wait_timer;
long c_wait_timer;
long d_wait_timer;
long e_wait_timer;
long a_wait_timer2;
long b_wait_timer2;
long c_wait_timer2;
long d_wait_timer2;
long e_wait_timer2;
byte min_state;
byte max_state;


byte a_state;
byte b_state;
byte c_state;
byte d_state;
byte e_state;
byte a_state2;
byte b_state2;
byte c_state2;
byte d_state2;
byte e_state2;
bool a_state_ack;
bool b_state_ack;
bool c_state_ack;
bool d_state_ack;
bool e_state_ack;
bool a_state_ack2;
bool b_state_ack2;
bool c_state_ack2;
bool d_state_ack2;
bool e_state_ack2;

// stepper acceleration
long current_delay = 2000;
long current_step = 0;


// configuration values
struct config_t
{
  unsigned long coilSteps;
  bool coilInvert;
  unsigned long guideSteps;
  bool guideInvert;
  unsigned long wireSize;   // micrometers
  unsigned long coilDiam;   // micrometers
  unsigned long coilLength; // micrometers
  unsigned long coilTurns;
  unsigned long coilSpeed;
  bool coilCalc;
  float coilRatio;
  unsigned long coilHome;   // micrometers
} conf;


// Connect via i2c, default address #0 (A0-A2 not jumpered)
LiquidCrystal lcd(0);



void setup()
{
  Serial.begin(BAUDRATE);

  pinMode(min_stop, INPUT);
  pinMode(max_stop, INPUT);

  pinMode(led, OUTPUT);

  pinMode(button_a, INPUT);           // set pin to input
  digitalWrite(button_a, HIGH);       // turn on pullup resistors
  pinMode(button_b, INPUT);           // set pin to input
  digitalWrite(button_b, HIGH);       // turn on pullup resistors
  pinMode(button_c, INPUT);           // set pin to input
  digitalWrite(button_c, HIGH);       // turn on pullup resistors
  pinMode(button_d, INPUT);           // set pin to input
  digitalWrite(button_d, HIGH);       // turn on pullup resistors
  pinMode(button_e, INPUT);           // set pin to input
  digitalWrite(button_e, HIGH);       // turn on pullup resistors

  pinMode(stepper_a_enable, OUTPUT);
  pinMode(stepper_a_direction, OUTPUT);
  pinMode(stepper_a_step, OUTPUT);
  pinMode(stepper_b_enable, OUTPUT);
  pinMode(stepper_b_direction, OUTPUT);
  pinMode(stepper_b_step, OUTPUT);
  
  digitalWrite(stepper_a_enable, HIGH);      // HIGH = disabled
  digitalWrite(stepper_a_direction, HIGH);  // HIGH = forward
  digitalWrite(stepper_a_step, LOW);

  digitalWrite(stepper_b_enable, HIGH);      // HIGH = disabled
  digitalWrite(stepper_b_direction, HIGH);  // HIGH = forward
  digitalWrite(stepper_b_step, LOW);


  // set up the LCD's number of rows and columns: 
  lcd.begin(20, 4);

  // set PWM speed
  TCCR0B = 0x01;   // Timer 0: PWM 5 &  6 @ 16 kHz

  /*

  If you change TCCR0B, it affects millis() and delay(). 
  They will count time faster or slower than normal if you
  change the TCCR0B settings. Below is the adjustment factor
  to maintain consistent behavior of these functions:

  Default: delay(1000) or 1000 millis() ~ 1 second

  0x01: delay(64000) or 64000 millis() ~ 1 second
  0x02: delay(8000) or 8000 millis() ~ 1 second
  0x03: is the default
  0x04: delay(250) or 250 millis() ~ 1 second
  0x05: delay(62) or 62 millis() ~ 1 second
  (Or 63 if you need to round up. The number is actually 62.5)

  */

  // TCCR1B = 0x05;   // Timer 0: PWM 9 &  10 @ 30.64 kHz
  // TCCR1B = 0x04;   // Timer 0: PWM 9 &  10 @ 122.55 kHz
  // TCCR1B = 0x03;   // Timer 0: PWM 9 &  10 @ 490.20 kHz
  // TCCR1B = 0x02;   // Timer 0: PWM 9 &  10 @ 3921.16 kHz
  // TCCR1B = 0x01;   // Timer 0: PWM 9 &  10 @ 31372.55 kHz
  // TCCR2B = 0x01;   // Timer 0: PWM 11 &  3 @ 32 kHz

  // define menu structure
  mm.add_menu(&manual);
  manual.add_item(&manual_stepper,&menu_stepper_enable);
  manual.add_item(&manual_jogcoil,&menu_jog_coil);
  manual.add_item(&manual_jogguide,&menu_jog_guide);

  mm.add_menu(&settings);
  settings.add_item(&settings_coilsteps,&menu_settings_coilsteps);
  settings.add_item(&settings_coilinvert,&menu_settings_coilinvert);
  settings.add_item(&settings_guidesteps,&menu_settings_guidesteps);
  settings.add_item(&settings_guideinvert,&menu_settings_guideinvert);
  settings.add_item(&settings_coilhome,&menu_job_coilhome);
  settings.add_item(&settings_home,&menu_home_guide);

  mm.add_menu(&job);
  job.add_item(&job_help,&menu_job_help);
  job.add_item(&job_wiredia,&menu_job_wiredia);
  job.add_item(&job_coildia,&menu_job_coildia);
  job.add_item(&job_coilhome,&menu_job_coilhome);
  job.add_item(&job_coilcalc,&menu_job_calc);
  job.add_item(&job_coilratio,&menu_job_ratio);
  job.add_item(&job_length,&menu_job_length);
  job.add_item(&job_turns,&menu_job_turns);
  job.add_item(&job_speed,&menu_job_speed);
  job.add_item(&job_home,&menu_home_guide);
  job.add_item(&job_start,&menu_job_start);

  mm.add_item(&job_start,&menu_job_start);

  ms.set_root_menu(&mm);

  lcd.setBacklight(LOW);
  delay(16000);
  lcd.setBacklight(HIGH);
  
  // Print a message to the lcd.
  lcd.clear();

  String LCDScroll = "                   Project Icarus  ";
  for(int i=0 ; i < 20 ; i++)
  {
    lcd.setCursor(0, 0);
    lcd.print(LCDScroll.substring(i,i+20));
    delay(1200);
  }  

  // read eeprom settings
  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf));

/*
  // set to initial sane values and save
  conf.coilSteps = 1600;
  conf.coilInvert = 0;
  conf.guideSteps = 320;
  conf.guideInvert = 0;
  conf.wireSize = 700;      // micrometers
  conf.coilDiam = 114070;   // micrometers
  conf.coilLength = 570350; // micrometers
  conf.coilTurns = 814;
  conf.coilSpeed = 20;
  conf.coilCalc = 1;
  conf.coilRatio = 5.00;
  conf.coilHome = 24500;    // micrometers

  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
*/

  delay(32000);

  lcd.clear();
  delay(16000);
  displayMenu();
}



void loop()
{
  check_buttons();

  if(a_state && !a_state_ack)
  {
    // center
    a_state_ack = 1;
    ms.select(0);
    displayMenu();
  }
  if(b_state && !b_state_ack)
  {
    // down
    b_state_ack = 1;
    ms.next();
    displayMenu();
  }
  if(c_state && !c_state_ack)
  {
    // up
    c_state_ack = 1;
    ms.prev();
    displayMenu();
  }
  if(d_state && !d_state_ack)
  {
    // left
    d_state_ack = 1;
    ms.back();
    displayMenu();
  }

  delay(1000);               // wait
}

void check_buttons()
{
  // end stop code
  min_state = digitalRead(min_stop);
  max_state = digitalRead(max_stop);

  // button debounce code
  
  int a_read = invert_buttons ^ digitalRead(button_a);
  if(a_read == a_state)
  {
    a_wait_timer = millis(); 
  }
  else if(a_read != a_state and (millis() - a_wait_timer) > DEBOUNCE_TIME)
  {
    a_state = a_read;
    a_state_ack = 0;
  }

  if(a_read == a_state2)
  {
    a_wait_timer2 = millis(); 
  }
  else if(a_read != a_state2 and (millis() - a_wait_timer2) > LONG_TIME)
  {
    a_state2 = a_read;
    a_state_ack2 = 0;
  }

  int b_read = invert_buttons ^ digitalRead(button_b);
  if(b_read == b_state)
  {
    b_wait_timer = millis(); 
  }
  else if(b_read != b_state and (millis() - b_wait_timer) > DEBOUNCE_TIME)
  {
    b_state = b_read;
    b_state_ack = 0;
  }

  if(b_read == b_state2)
  {
    b_wait_timer2 = millis(); 
  }
  else if(b_read != b_state2 and (millis() - b_wait_timer2) > LONG_TIME)
  {
    b_state2 = b_read;
    b_state_ack2 = 0;
  }

  int c_read = invert_buttons ^ digitalRead(button_c);
  if(c_read == c_state)
  {
    c_wait_timer = millis(); 
  }
  else if(c_read != c_state and (millis() - c_wait_timer) > DEBOUNCE_TIME)
  {
    c_state = c_read;
    c_state_ack = 0;
  }

  if(c_read == c_state2)
  {
    c_wait_timer2 = millis(); 
  }
  else if(c_read != c_state2 and (millis() - c_wait_timer2) > LONG_TIME)
  {
    c_state2 = c_read;
    c_state_ack2 = 0;
  }

  int d_read = invert_buttons ^ digitalRead(button_d);
  if(d_read == d_state)
  {
    d_wait_timer = millis(); 
  }
  else if(d_read != d_state and (millis() - d_wait_timer) > DEBOUNCE_TIME)
  {
    d_state = d_read;
    d_state_ack = 0;
  }

  int e_read = invert_buttons ^ digitalRead(button_e);
  if(e_read == e_state)
  {
    e_wait_timer = millis(); 
  }
  else if(e_read != e_state and (millis() - e_wait_timer) > DEBOUNCE_TIME)
  {
    e_state = e_read;
    e_state_ack = 0;
  }
}

void menu_stepper_enable(MenuItem* p_menu_item)
{
    stepper_enable(1 ^ (stepper_a_ena & 1));
}

void stepper_enable(byte enable)
{
  /*
    options
    0 = turn off and save
    1 = turn on and save
    3 = turn off
    4 = turn on
    5 = revert to saved
    
    bits
    1 = operating state
    2 = saved state
    
    Set bit
    number |= 1 << x;
    
    Clear bit
    number &= ~(1 << x);
    
    Toggle bit
    number ^= 1 << x;
    
    Check bit
    bit = number & (1 << x);
  */
  if(enable == 0)
  {
    // set and save bit 1 and 2
    stepper_a_ena = 0;
  }
  else if(enable == 1)
  {
    stepper_a_ena = 3;
  }
  else if(enable == 3)
  {
    // turn off 1 bit
    stepper_a_ena &= ~(1 << 0);
  }
  else if(enable == 4)
  {
    // turn on 1 bit
    stepper_a_ena |= 1 << 0;
  }
  else if(enable == 5)
  {
    byte stepper_saved = stepper_a_ena & 2;
    if(stepper_saved)
    {
      stepper_a_ena |= 1 << 0;
    }
    else
    {
      stepper_a_ena &= ~(1 << 0);
    }
  }

  if(stepper_a_ena & 1)
  {
    // enable steppers
    digitalWrite(led, HIGH);
    digitalWrite(stepper_a_enable, LOW);      // HIGH = disabled
    digitalWrite(stepper_b_enable, LOW);      // HIGH = disabled
  }
  else if(!stepper_a_ena & 1)
  {
    // disable steppers
    digitalWrite(led, LOW);
    digitalWrite(stepper_a_enable, HIGH);      // HIGH = disabled
    digitalWrite(stepper_b_enable, HIGH);      // HIGH = disabled
  }
}


void displayMenu()
{
  lcd.setCursor(0,0);
  // Display the menu
  Menu const* cp_menu = ms.get_current_menu();

  //lcd.print("Current menu name: ");
  lcd.print(cp_menu->get_name());
  
  lcd.setCursor(0,1);

  MenuComponent const* cp_menu_sel = cp_menu->get_selected();

  int menu_components = cp_menu->get_num_menu_components();
  int current_item = cp_menu->get_cur_menu_component_num();

  // calculate start and end items
  int start = current_item - 1;
  if(start < 0)
  {
    start = 0;
  }
  if(start + 3 > menu_components)
  {
    start = menu_components - 3;
  }
  int max_item = start + 3;
  if(max_item > menu_components)
  {
    max_item = menu_components;
  }

  line = 1;
  for (int i = start; i < max_item; ++i)
  {
    lcd.setCursor(0,line);
    
    MenuComponent const* cp_m_comp = cp_menu->get_menu_component(i);

    if (cp_menu_sel == cp_m_comp)
    {
      lcd.print(">");
      cursor_line = line;
    }
    else
    {
      lcd.print(" ");
    }
    
    lcd.print(cp_m_comp->get_name());
    

    // display setting information    
    if(cp_m_comp->get_name() == ENABLE)
    {
      print_bool(stepper_a_ena & 1);
    }
    else if(cp_m_comp->get_name() == COIL_STEPS)
    {
      print_long(conf.coilSteps);
    }
    else if(cp_m_comp->get_name() == COIL_INV)
    {
      print_bool(conf.coilInvert);
    }
    else if(cp_m_comp->get_name() == GUIDE_INV)
    {
      print_bool(conf.guideInvert);
    }
    else if(cp_m_comp->get_name() == GUIDE_STEPS)
    {
      print_long(conf.guideSteps);
    }
    else if(cp_m_comp->get_name() == WIRE_SIZE)
    {
      print_long(conf.wireSize);
    }
    else if(cp_m_comp->get_name() == COIL_DIAM)
    {
      print_long(conf.coilDiam);
    }
    else if(cp_m_comp->get_name() == COIL_HOME)
    {
      print_long(conf.coilHome);
    }
    else if(cp_m_comp->get_name() == COIL_CALC)
    {
      print_bool(conf.coilCalc);
    }
    else if(cp_m_comp->get_name() == COIL_RATIO)
    {
      print_float(conf.coilRatio);
    }
    else if(cp_m_comp->get_name() == COIL_LEN)
    {
      print_long(conf.coilLength);
    }
    else if(cp_m_comp->get_name() == COIL_TURNS)
    {
      print_long(conf.coilTurns);
    }
    else if(cp_m_comp->get_name() == COIL_SPEED)
    {
      print_long(conf.coilSpeed);
    }
      
    line++;
  }
}

void print_bool(bool bool_val)
{
  lcd.setCursor(17,line);
  if(bool_val)
  {
    lcd.print("Yes");
  }
  else
  {
    lcd.print(" No");
  }
}

void print_long(long long_val)
{
  char buf[16];
  
  ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
    
  lcd.setCursor(20-strlen(buf),line);
  lcd.print(buf);
}

void print_float(float float_val)
{
  char buf[16];
  
  //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
  dtostrf(float_val,4,2,buf);
    
  lcd.setCursor(20-strlen(buf),line);
  lcd.print(buf);
}

long edit_long(long long_val)
{
  long incr = 1;
  byte incr_count = 0;
  long saved_long_val = long_val;
  char buf[16];
  line = cursor_line;
  
  ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
    
  lcd.setCursor(0, line);
  lcd.print(" ");

  lcd.setCursor(18-strlen(buf),cursor_line);
  lcd.print(" >");

  while(1)
  {
    check_buttons();

    if(a_state && !a_state_ack)
    {
      // center
      a_state_ack = 1;
      break;
    }
    if(b_state2 && !b_state_ack2)
    {
      incr_count++;
      if(incr_count > 8)
      {
        incr = incr * 10;
        incr_count = 0;
      }

      if(long_val > incr)
      {
        long_val = long_val - incr;
      }
      else
      {
        ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
        lcd.setCursor(19-strlen(buf),cursor_line);
        for(int i=0; i<=strlen(buf); i++)
        {
          lcd.print(" ");
        }
        long_val = 0;
      }

      b_state2 = 0;
      b_state_ack2 = 1;
     
      b_wait_timer2 = millis(); 

      ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_long(long_val);
    }
    if(b_state == 2 && !b_state_ack)
    {
      if(long_val > 10)
      {
        long_val = long_val - 10;
      }
      else
      {
        ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
        lcd.setCursor(19-strlen(buf),cursor_line);
        for(int i=0; i<=strlen(buf); i++)
        {
          lcd.print(" ");
        }
        long_val = 0;
      }
      b_state_ack = 1;

      ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_long(long_val);
    }
    if(b_state && !b_state_ack)
    {
      // down
      if(long_val > 0)
      {
        long_val--;
      }
      b_state_ack = 1;

      incr = 1;
      incr_count = 0;      

      ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_long(long_val);
    }
    if(c_state2 && !c_state_ack2)
    {
      incr_count++;
      if(incr_count > 8)
      {
        incr = incr * 10;
        incr_count = 0;
      }
      long_val = long_val + incr;
      c_state2 = 0;
      c_state_ack2 = 1;
     
      c_wait_timer2 = millis(); 

      ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_long(long_val);
    }
    if(c_state && !c_state_ack)
    {
      // up
      long_val++;
      c_state_ack = 1;
      incr = 1;
      incr_count = 0;      

      ltoa(long_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_long(long_val);
    }
    if(d_state && !d_state_ack)
    {
      // left
      d_state_ack = 1;
      long_val = saved_long_val;
      break;
    }
  }
  
  return long_val; 
}

float edit_float(float float_val)
{
  float incr = .01;
  byte incr_count = 0;
  float saved_float_val = float_val;
  char buf[16];
  line = cursor_line;
  
  // ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
  dtostrf(float_val,4,2,buf);
    
  lcd.setCursor(0, line);
  lcd.print(" ");

  lcd.setCursor(18-strlen(buf),cursor_line);
  lcd.print(" >");

  while(1)
  {
    check_buttons();

    if(a_state && !a_state_ack)
    {
      // center
      a_state_ack = 1;
      break;
    }
    if(b_state2 && !b_state_ack2)
    {
      incr_count++;
      if(incr_count > 8)
      {
        incr = incr * 10;
        incr_count = 0;
      }

      if(float_val > incr)
      {
        float_val = float_val - incr;
      }
      else
      {
        //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
        dtostrf(float_val,4,2,buf);

        lcd.setCursor(19-strlen(buf),cursor_line);
        for(int i=0; i<=strlen(buf); i++)
        {
          lcd.print(" ");
        }
        float_val = 0;
      }

      b_state2 = 0;
      b_state_ack2 = 1;
     
      b_wait_timer2 = millis(); 

      //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      dtostrf(float_val,4,2,buf);

      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_float(float_val);
    }
    if(b_state == 2 && !b_state_ack)
    {
      if(float_val > 10)
      {
        float_val = float_val - 10;
      }
      else
      {
        //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
        dtostrf(float_val,4,2,buf);

        lcd.setCursor(19-strlen(buf),cursor_line);
        for(int i=0; i<=strlen(buf); i++)
        {
          lcd.print(" ");
        }
        float_val = 0;
      }
      b_state_ack = 1;

      //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      dtostrf(float_val,4,2,buf);

      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_float(float_val);
    }
    if(b_state && !b_state_ack)
    {
      // down
      incr = .01;
      incr_count = 0;      

      if(float_val > 0)
      {
        float_val = float_val - incr;
      }
      b_state_ack = 1;

      //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      dtostrf(float_val,4,2,buf);

      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_float(float_val);
    }
    if(c_state2 && !c_state_ack2)
    {
      incr_count++;
      if(incr_count > 8)
      {
        incr = incr * 10;
        incr_count = 0;
      }
      float_val = float_val + incr;
      c_state2 = 0;
      c_state_ack2 = 1;
     
      c_wait_timer2 = millis(); 

      //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      dtostrf(float_val,4,2,buf);

      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_float(float_val);
    }
    if(c_state && !c_state_ack)
    {
      // up
      c_state_ack = 1;
      incr = .01;
      incr_count = 0;      
      float_val = float_val + incr;

      //ltoa(float_val, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
      dtostrf(float_val,4,2,buf);
 
      lcd.setCursor(18-strlen(buf),cursor_line);
      lcd.print(" >");
      print_float(float_val);
    }
    if(d_state && !d_state_ack)
    {
      // left
      d_state_ack = 1;
      float_val = saved_float_val;
      break;
    }
  }
  
  return float_val; 
}

void delay_step(long steps, int max_delay_time, int delay_time, int step_amount, byte accel_mode)
{
  long accel_steps = (max_delay_time - delay_time) / step_amount;
  accel_steps = abs(accel_steps);
    
  if(current_step == 0)
  {
    current_delay = max_delay_time;
  }
  else if(current_step > (steps - accel_steps) and accel_mode & 1)
  {
    // decel
    if(current_delay < max_delay_time)
    {
      current_delay = current_delay + step_amount;
    }
  }
  else if(current_delay > delay_time and accel_mode & 2)
  {
    current_delay = current_delay - step_amount;
  }
  else if(current_delay < delay_time)
  {
    current_delay = current_delay + step_amount;
  }
  delayMicroseconds(current_delay);
  current_step++;
}


void menu_jog_coil(MenuItem* p_menu_item)
{
  byte jog_mode = 0;
  byte jog_dir = 0;
  byte jog_run = 0;
  long jog_speed = conf.coilSpeed;
  
  lcd.clear();
  lcd.print("Jog coil");

  line = 0;
  print_long(jog_speed);

  lcd.setCursor(10,0);
  lcd.print("  Man");


  lcd.setCursor(7, 1);
  lcd.print("^");
  lcd.print(" Fwd");
  lcd.setCursor(0, 2);
  lcd.print("Back < ");
  lcd.write(219);
  lcd.print(" Mode  >");
  lcd.setCursor(7, 3);
  lcd.print("v Rev   Stop");
  
  while(1)
  {
    check_buttons();
    if(d_state && !d_state_ack)
    {
      // left
      d_state_ack = 1;
      lcd.clear();
      break;
    }
    else if(a_state && !a_state_ack)
    {
      // center
      a_state_ack = 1;
      jog_mode++;
      if(jog_mode > 2)
      {
        jog_mode = 0;
      }
      lcd.setCursor(12,0);
      if(jog_mode == 0)
      {
        lcd.print("Man ");
      }
      else if(jog_mode == 1)
      {
        lcd.print("Cont");
      }
      else if(jog_mode == 2)
      {
        lcd.print("Fixd");
      }
    }
    else if(e_state && !e_state_ack)
    {
      // left
      e_state_ack = 1;
      lcd.setCursor(10,0);
      lcd.print(" ");
      jog_run = 0;
    }
    
    if(jog_mode == 0)
    {
      jog_run = 0;

      // Manual
      if(c_state)
      {
        stepper_enable(4);

        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("F");
        jog_dir = 1;

        digitalWrite(stepper_b_direction, conf.coilInvert ^ jog_dir);  // HIGH = forward
        current_step = 0;

        while(c_state)
        {
          check_buttons();
          digitalWrite(stepper_b_step, LOW);  
          delayMicroseconds(125);      // This delay time is close to top speed for this
          digitalWrite(stepper_b_step, HIGH); 
          delay_step(conf.guideSteps, 2005, 385, 10, 2);
        }
        current_step = 1;
        while(current_delay < 2000)
        {
          digitalWrite(stepper_b_step, LOW);  
          delayMicroseconds(125);      // This delay time is close to top speed for this
          digitalWrite(stepper_b_step, HIGH); 
          delay_step(82, 2005, 385, 20, 1);
        }
        lcd.setCursor(10,0);
        lcd.print(" ");
        stepper_enable(5);
      }
      if(b_state)
      {
        stepper_enable(4);

        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("R");
        jog_dir = 0;

        digitalWrite(stepper_b_direction, conf.coilInvert ^ jog_dir);  // HIGH = forward
        current_step = 0;

        while(b_state)
        {
          check_buttons();
          digitalWrite(stepper_b_step, LOW);  
          delayMicroseconds(125);      
          digitalWrite(stepper_b_step, HIGH); 
          delay_step(conf.coilSteps, 2005, 385, 10, 2);
        }
        current_step = 1;
        while(current_delay < 2000)
        {
          digitalWrite(stepper_b_step, LOW);  
          delayMicroseconds(125);      // This delay time is close to top speed for this
          digitalWrite(stepper_b_step, HIGH); 
          delay_step(82, 2005, 385, 20, 1);
        }
        lcd.setCursor(10,0);
        lcd.print(" ");
        stepper_enable(5);
      }
    }
    else if(jog_mode == 1)
    {
      // Continuous
      if(!c_state_ack and c_state and (!jog_run or (jog_run and !jog_dir)))
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("F");
        jog_dir = 1;
        jog_run = 1;
        current_step = 0;
        stepper_enable(4);

        digitalWrite(stepper_b_direction, conf.coilInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!c_state_ack and c_state and jog_run)
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }

      if(!b_state_ack and b_state and (!jog_run or (jog_run and jog_dir)))
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("R");
        jog_dir = 0;
        jog_run = 1;
        current_step = 0;
        stepper_enable(4);

        digitalWrite(stepper_b_direction, conf.coilInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!b_state_ack and b_state and jog_run)
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }
      
      if(jog_run)
      {
        digitalWrite(stepper_b_step, LOW);  
        delayMicroseconds(125);      // This delay time is close to top speed for this
        digitalWrite(stepper_b_step, HIGH); 
        delay_step(conf.coilSteps, 2005, 385, 10, 2);
      }
      else
      {
        current_step = 1;
        while(current_delay < 2000)
        {
          digitalWrite(stepper_b_step, LOW);  
          delayMicroseconds(125);      // This delay time is close to top speed for this
          digitalWrite(stepper_b_step, HIGH); 
          delay_step(82, 2005, 385, 20, 1);
        }
        stepper_enable(5);
      }
    }
    else if(jog_mode == 2)
    {
      // Fixed
      jog_run = 0;

      if(!c_state_ack and c_state and (!jog_run or (jog_run and !jog_dir)))
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("F");
        jog_dir = 1;
        jog_run = 1;
        stepper_enable(4);

        digitalWrite(stepper_b_direction, conf.coilInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!c_state_ack and c_state and jog_run)
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }

      if(!b_state_ack and b_state and (!jog_run or (jog_run and jog_dir)))
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("R");
        jog_dir = 0;
        jog_run = 1;
        stepper_enable(4);

        digitalWrite(stepper_b_direction, conf.coilInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!b_state_ack and b_state and jog_run)
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }
      
      if(jog_run)
      {
        current_step = 0;
        for(long i = 0 ; i < conf.coilSteps ; i++)
        {
          check_buttons();
          if(e_state && !e_state_ack)
          {
            // left
            e_state_ack = 1;
            lcd.setCursor(10,0);
            lcd.print(" ");
            jog_run = 0;
            break;
          }

          digitalWrite(stepper_b_step, LOW);  
          delayMicroseconds(125);
          digitalWrite(stepper_b_step, HIGH); 
          delay_step(conf.coilSteps, 2000, 385, 5, 3);
        }
        jog_run = 0;
        lcd.setCursor(10,0);
        lcd.print(" ");
        stepper_enable(5);
      }
    }
  }
}

void menu_jog_guide(MenuItem* p_menu_item)
{
  byte jog_mode = 0;
  byte jog_dir = 0;
  byte jog_run = 0;
  long jog_speed = conf.coilSpeed;
  
  lcd.clear();
  lcd.print("Jog guide");

  line = 0;
  print_long(jog_speed);

  lcd.setCursor(10,0);
  lcd.print("  Man");


  lcd.setCursor(7, 1);
  lcd.print("^");
  lcd.print(" Fwd");
  lcd.setCursor(0, 2);
  lcd.print("Back < ");
  lcd.write(219);
  lcd.print(" Mode  >");
  lcd.setCursor(7, 3);
  lcd.print("v Rev   Stop");
  
  while(1)
  {
    check_buttons();
    
    if(d_state && !d_state_ack)
    {
      // left
      d_state_ack = 1;
      lcd.clear();
      break;
    }
    else if(a_state && !a_state_ack)
    {
      // center
      a_state_ack = 1;
      jog_mode++;
      if(jog_mode > 2)
      {
        jog_mode = 0;
      }
      lcd.setCursor(12,0);
      if(jog_mode == 0)
      {
        lcd.print("Man ");
      }
      else if(jog_mode == 1)
      {
        lcd.print("Cont");
      }
      else if(jog_mode == 2)
      {
        lcd.print("Fixd");
      }
    }
    else if(e_state && !e_state_ack)
    {
      // left
      e_state_ack = 1;
      lcd.setCursor(10,0);
      lcd.print(" ");
      jog_run = 0;
    }
    
    if(jog_mode == 0)
    {
      jog_run = 0;

      // Manual
      if(c_state && max_state)
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("F");
        jog_dir = 1;
        stepper_enable(4);

        digitalWrite(stepper_a_direction, conf.guideInvert ^ jog_dir);  // HIGH = forward
        current_step = 0;

        while(c_state && max_state)
        {
          check_buttons();
          digitalWrite(stepper_a_step, LOW);  
          delayMicroseconds(125);      // This delay time is close to top speed for this
          digitalWrite(stepper_a_step, HIGH); 
          delay_step(conf.guideSteps, 2005, 385, 10, 2);
        }
        if(max_state)
        {
          current_step = 1;
          while(current_delay < 2000)
          {
            digitalWrite(stepper_a_step, LOW);  
            delayMicroseconds(125);      // This delay time is close to top speed for this
            digitalWrite(stepper_a_step, HIGH); 
            delay_step(82, 2005, 385, 20, 1);
          }
          stepper_enable(5);
        }
        else
        {
          jog_run = 0;
          current_delay = 2000;
        }
        lcd.setCursor(10,0);
        lcd.print(" ");
      }
      if(b_state && min_state)
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("R");
        jog_dir = 0;
        stepper_enable(4);

        digitalWrite(stepper_a_direction, conf.guideInvert ^ jog_dir);  // HIGH = forward
        current_step = 0;

        while(b_state && min_state)
        {
          check_buttons();
          digitalWrite(stepper_a_step, LOW);  
          delayMicroseconds(125);      
          digitalWrite(stepper_a_step, HIGH); 
          delay_step(conf.guideSteps, 2005, 385, 10, 2);
        }
        if(min_state)
        {
          current_step = 1;
          while(current_delay < 2000)
          {
            digitalWrite(stepper_a_step, LOW);  
            delayMicroseconds(125);      // This delay time is close to top speed for this
            digitalWrite(stepper_a_step, HIGH); 
            delay_step(82, 2005, 385, 20, 1);
          }
          stepper_enable(5);
        }
        else
        {
          jog_run = 0;
          current_delay = 2000;
        }
        lcd.setCursor(10,0);
        lcd.print(" ");
      }
    }
    else if(jog_mode == 1)
    {
      // Continuous
      if(!c_state_ack and c_state and (!jog_run or (jog_run and !jog_dir)))
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("F");
        jog_dir = 1;
        jog_run = 1;
        current_step = 0;
        stepper_enable(4);

        digitalWrite(stepper_a_direction, conf.guideInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!c_state_ack and c_state and jog_run)
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }

      if(!b_state_ack and b_state and (!jog_run or (jog_run and jog_dir)))
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("R");
        jog_dir = 0;
        jog_run = 1;
        current_step = 0;
        stepper_enable(4);

        digitalWrite(stepper_a_direction, conf.guideInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!b_state_ack and b_state and jog_run)
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }
      
      if(jog_run && min_state && max_state)
      {
        digitalWrite(stepper_a_step, LOW);  
        delayMicroseconds(125);      // This delay time is close to top speed for this
        digitalWrite(stepper_a_step, HIGH); 
        delay_step(conf.guideSteps, 2005, 385, 10, 2);
      }
      else if(!min_state || !max_state)
      {
        // stop immediately
        jog_run = 0;
        current_delay = 2000;
      }
      else
      {
        current_step = 1;
        while(current_delay < 2000)
        {
          digitalWrite(stepper_a_step, LOW);  
          delayMicroseconds(125);      // This delay time is close to top speed for this
          digitalWrite(stepper_a_step, HIGH); 
          delay_step(82, 2005, 385, 20, 1);
        }
        stepper_enable(5);

      }
    }
    else if(jog_mode == 2)
    {
      // Fixed
      jog_run = 0;

      if(!c_state_ack and c_state and (!jog_run or (jog_run and !jog_dir)))
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("F");
        jog_dir = 1;
        jog_run = 1;
        stepper_enable(4);

        digitalWrite(stepper_a_direction, conf.guideInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!c_state_ack and c_state and jog_run)
      {
        c_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }

      if(!b_state_ack and b_state and (!jog_run or (jog_run and jog_dir)))
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print("R");
        jog_dir = 0;
        jog_run = 1;
        stepper_enable(4);

        digitalWrite(stepper_a_direction, conf.guideInvert ^ jog_dir);  // HIGH = forward
      }
      else if(!b_state_ack and b_state and jog_run)
      {
        b_state_ack = 1;
        lcd.setCursor(10,0);
        lcd.print(" ");
        jog_run = 0;
      }
      
      if(jog_run && ((!jog_dir && min_state) || (jog_dir && max_state)))
      {
        current_step = 0;
        for(long i = 0 ; i < conf.guideSteps ; i++)
        {
          check_buttons();
          if((e_state && !e_state_ack) || (!jog_dir && !min_state) || (jog_dir && !max_state))
          {
            // left
            e_state_ack = 1;
            lcd.setCursor(10,0);
            lcd.print(" ");
            jog_run = 0;
            break;
          }

          digitalWrite(stepper_a_step, LOW);  
          delayMicroseconds(125);
          digitalWrite(stepper_a_step, HIGH); 
          delay_step(conf.guideSteps, 1000, 385, 5, 3);
        }
        jog_run = 0;
        current_delay = 2000;
        lcd.setCursor(10,0);
        lcd.print(" ");
        stepper_enable(5);
      }
    }
  }
}

void menu_settings_coilsteps(MenuItem* p_menu_item)
{
  conf.coilSteps = edit_long(conf.coilSteps);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_settings_coilinvert(MenuItem* p_menu_item)
{
  conf.coilInvert = conf.coilInvert ^ 1;
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_settings_guidesteps(MenuItem* p_menu_item)
{
  conf.guideSteps = edit_long(conf.guideSteps);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_settings_guideinvert(MenuItem* p_menu_item)
{
  conf.guideInvert = conf.guideInvert ^ 1;
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_home_guide(MenuItem* p_menu_item)
{
  // home guide
  lcd.setCursor(0,3);
  lcd.print("Homing Guide        ");

  current_step = 0;
  stepper_enable(4);

  digitalWrite(stepper_a_direction, conf.guideInvert ^ 0);  // HIGH = forward

  while(min_state)
  {
    check_buttons();
    if((e_state && !e_state_ack) || (!min_state) || (!max_state))
    {
      // left
      e_state_ack = 1;
      break;
    }
    
    digitalWrite(stepper_a_step, LOW);  
    delayMicroseconds(125);      // This delay time is close to top speed for this
    digitalWrite(stepper_a_step, HIGH); 
    delay_step(conf.guideSteps, 2005, 385, 10, 2);
  }
  current_delay = 2000;
  
  digitalWrite(stepper_a_direction, conf.guideInvert ^ 1);  // HIGH = forward
  
  unsigned long total_steps = conf.coilHome * conf.guideSteps / 1000;
  
  
  current_step = 0;
  for(unsigned long i = 0 ; i < total_steps ; i++)
  {
    check_buttons();
    if((e_state && !e_state_ack) || !max_state)
    {
      // left
      e_state_ack = 1;
      break;
    }
    digitalWrite(stepper_a_step, LOW);  
    delayMicroseconds(125);
    digitalWrite(stepper_a_step, HIGH); 
    delay_step(total_steps, 1000, 385, 5, 3);
  }
  
  current_delay = 2000;
  stepper_enable(5);
  
  lcd.setCursor(0,3);
  lcd.print("              ");
}

void menu_job_wiredia(MenuItem* p_menu_item)
{
  conf.wireSize = edit_long(conf.wireSize);
  calc_coil(0);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_job_help(MenuItem* p_menu_item)
{
  
}

void menu_job_coildia(MenuItem* p_menu_item)
{
  conf.coilDiam = edit_long(conf.coilDiam);
  calc_coil(0);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_job_coilhome(MenuItem* p_menu_item)
{
  conf.coilHome = edit_long(conf.coilHome);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_job_length(MenuItem* p_menu_item)
{
  conf.coilLength = edit_long(conf.coilLength);
  calc_coil(1);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_job_turns(MenuItem* p_menu_item)
{
  conf.coilTurns = edit_long(conf.coilTurns);
  calc_coil(2);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_job_speed(MenuItem* p_menu_item)
{
  conf.coilSpeed = edit_long(conf.coilSpeed);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_job_calc(MenuItem* p_menu_item)
{
  conf.coilCalc = conf.coilCalc ^ 1;
  calc_coil(0);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void menu_job_ratio(MenuItem* p_menu_item)
{
  conf.coilRatio = edit_float(conf.coilRatio);
  calc_coil(0);
  eeprom_write_block((const void*)&conf, (void*)0, sizeof(conf));
}

void calc_coil(byte calc_type)
{
  // calculate the coil size based on secondary diameter, ratio, and wire size
  if(conf.coilCalc)
  {
    if(calc_type == 0)
    {
      // using wire diam, coil diam, and ratio
      conf.coilLength = conf.coilDiam * conf.coilRatio;
      conf.coilTurns = conf.coilLength / conf.wireSize;
    }
    else if(calc_type == 1)
    {
      // using wire diam, coil diam, and lenght
      float i = (float) (conf.coilLength / conf.coilDiam) * 100 + 0.5;
      conf.coilRatio = i / 100;
      conf.coilTurns = conf.coilLength / conf.wireSize;
    }
    else if(calc_type == 2)
    {
      // using wire diam, coil diam, and turns
      conf.coilLength = conf.coilTurns * conf.wireSize;
      float i = (float) (conf.coilLength / conf.coilDiam) * 100 + 0.5;

      conf.coilRatio = i / 100;
    }
  }
}

void menu_job_start(MenuItem* p_menu_item)
{
  /*
      Coil winding subroutine        
  */

  int max_speed = 50;
  int set_speed = 200;
  int min_speed = 2000;
  int saved_set_speed = set_speed;

  byte pause = 0;
  long guide_steps = 0;
  char buf[16];
  ltoa(conf.coilTurns, buf, 10);  // 10 is the base value not the size - look up ltoa for avr
  int len_buf = strlen(buf);
  
  char buf2[16];

  lcd.clear();
  lcd.print("Winding");
  lcd.setCursor(0,1);
  lcd.print("     Turn:");
  lcd.setCursor(0,2);
  lcd.print("[] Pause  > Stop");

  byte end_job = 0;
  current_step = 0;

  ltoa(0, buf2, 10);  // 10 is the base value not the size - look up ltoa for avr
  lcd.setCursor(11,1);
  lcd.print(buf2);
  lcd.setCursor(11+len_buf,1);
  lcd.print("/");
  lcd.print(buf);

  if(!stepper_a_ena)
  {
    stepper_enable(1);
  }

  digitalWrite(stepper_a_direction, conf.guideInvert ^ 1);  // HIGH = forward
  digitalWrite(stepper_b_direction, conf.coilInvert ^ 1);  // HIGH = forward


  // calculate our loop value
  unsigned long coil_steps = (conf.wireSize * conf.guideSteps) / 1000; // use guide steps for coil
  unsigned long total_coil_steps = conf.coilSteps * conf.coilTurns; // use guide steps for coil
  unsigned long loop_steps = conf.coilSteps * guide_steps;

  unsigned long loop_counter = 0;
  unsigned long guide_counter = 0;
  unsigned long coil_counter = 0;
  unsigned long foo_counter = 0;
  unsigned long winding_counter = 0;
  unsigned long intersections = 0;

  for(long x = 0 ; x < total_coil_steps ; x = x + conf.coilSteps)
  {
    if(x % coil_steps)
    {
      intersections++;
    }
  }
  
  total_coil_steps = total_coil_steps + (conf.coilTurns * conf.wireSize / 1000 * conf.guideSteps) - intersections - conf.coilSteps;
    
  while(winding_counter < conf.coilTurns)
  {
    
    loop_counter++;
    guide_counter++;
    coil_counter++;
    int delaynow = 0;
    if(guide_counter >= conf.coilSteps)
    {
      digitalWrite(stepper_a_step, LOW);  
      delaynow = 1;
    }

    if(coil_counter >= coil_steps)
    {
      digitalWrite(stepper_b_step, LOW);  
      delaynow = 1;
    }

    if(delaynow)
    {
      delayMicroseconds(max_speed);
    }

    if(guide_counter >= conf.coilSteps)
    {
      guide_counter = 0;
      digitalWrite(stepper_a_step, HIGH); 
    }

    if(coil_counter >= coil_steps)
    {

      // check input
      check_buttons();
      if(e_state && !e_state_ack)
      {
        // left - stop
        e_state_ack = 1;
        break;
      }

      if(a_state && !a_state_ack)
      {
        // left - pause
        a_state_ack = 1;

        // Timing is very crucial when you are running stepper and want a smooth speed,
		// so much of the informational LCD information is disabled while winding to prevent
		// undesired micro-stops
		
		//        lcd.setCursor(0,3);
        if(!pause)
        {
          //          lcd.print("Paused");
          pause = 1;
          saved_set_speed = set_speed;
          set_speed = min_speed;
        }
        else
        {
          //          lcd.print("      ");
          pause = 0;
          set_speed = saved_set_speed;
        } 
      }
      
      if(pause && current_delay == set_speed)
      {
        while(1)
        {
          check_buttons();
          if(a_state && !a_state_ack)
          {
            // left - pause
            a_state_ack = 1;
            pause = 0;
            set_speed = saved_set_speed;
            break;
          }
          if(e_state && !e_state_ack)
          {
            break;
          }
        }
        if(e_state && !e_state_ack)
        {
          // left - stop
          e_state_ack = 1;
          break;
        }
      }
      
      if(b_state && !b_state_ack)
      {
        // down
        b_state_ack = 1;
        set_speed = set_speed * 2;
        if(set_speed > min_speed)
        {
          set_speed = min_speed;
        }
        lcd.setCursor(10,0);
        lcd.print(set_speed);
        lcd.print("  ");
      }

      if(c_state && !c_state_ack)
      {
        // up
        c_state_ack = 1;
        set_speed = set_speed / 2;
        if(set_speed < max_speed)
        {
          set_speed = max_speed;
        }
        lcd.setCursor(10,0);
        lcd.print(set_speed);
        lcd.print("  ");
      }


      coil_counter = 0;
      foo_counter++;
      if(foo_counter >= conf.coilSteps)
      {
        foo_counter = 0;
        winding_counter++;
        lcd.setCursor(11,1);
        lcd.print(winding_counter);
      }
      digitalWrite(stepper_b_step, HIGH); 
    }

    if(delaynow)
    {
      delay_step(total_coil_steps, min_speed, set_speed, 5, 3);
    }
  }

  lcd.clear();
  lcd.print("Winding");
  lcd.setCursor(0,1);
  lcd.print("     Complete");
  lcd.setCursor(0,2);
  lcd.print("< Return");

  while(1)
  {
    check_buttons();
    if(d_state && !d_state_ack)
    {
      // left - stop
      d_state_ack = 1;
      break;
    }
  }

  stepper_enable(0);
}

void print_status(long turn_number)
{
  /*
     this function largely abandoned due to it taking too
	 much time.  I'm sure the code could be rewritten to be
	 a bit more efficient about things, but we're not there
	 just yet.
  
      12345678901234567890
     [Winding         65% ]
     [     Turn:   345/814]
     [ Position:    234234]
     [   Meters:     12345]
  */
  char buf[16];
  char buf2[16];

}
