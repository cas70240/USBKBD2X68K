//----------------------------------------------------------------------
// USBKBD2X68K for Arduino uno(ATMEGA328p compatible)
// X68000のキーボードコネクタにUSBキーボードとUSBマウスを接続
//----------------------------------------------------------------------
// 2020.11.09 Ver.0.1   とりあえず動いた版
// 2020.12.30 Ver.0.2   キーリピート間隔の実装（しかし実機と挙動が異なる）
// 2021.01.02 Ver.0.3   MOUSE_MODEでマウスを使用しないときはOFF出来るようにした
// 2021.02.14 Ver.0.4   MOUSE_MODE廃止,MOUSE CONTROLを実装,記号入力/登録/コード入力をテンキーに割り当て
//----------------------------------------------------------------------
// このスケッチのコンパイルには以下のライブラリが必要です.
//  ・USB_Host_Shield_2.0 (https://github.com/felis/USB_Host_Shield_2.0)
//  ・MsTimer2 (http://playground.arduino.cc/Main/MsTimer2)
//----------------------------------------------------------------------
// キーボードコネクタ配線(本体側)
//
//   7 6 5
//  4     3
//   2   1
//
//  本体側             Arduino側
//  -------------------------------------------
//  1:Vcc2 5V(out) -> 5V
//  2:MSDATA(in)   <- A0(14) softwareSerial TX
//  3:KEYRxD(in)   <- D1(1) Serial TX 
//  4:KEYTxD(out)  -> D0(0) Serial RX
//  5:READY(out)   -> D7(7)
//  6:REMOTE(in)
//  7:GND(--)      -- GND
//----------------------------------------------------------------------

#define MYDEBUG      0

#include <EEPROM.h>
//#include <BTHID.h>
#include <hidboot.h>
#include <usbhub.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>

#define RET_OK        0x75
#define RET_OK_EXIT   0x76
#define RET_D1_OK     0x77
#define RET_D2A_OK    0x78
#define RET_D2B_OK    0x79
#define RET_D2S_OK    0x7A
#define RET_ERROR     0x7B

#define MS_TX      14    // Mouse TX
#define MS_RX      15    // Mouse RX

SoftwareSerial MSSerial(MS_RX, MS_TX); // RX(KEYTxD/MSCTRL), TX(KEYRxD)

#define LOBYTE(x) ((char*)(&(x)))[0]
#define HIBYTE(x) ((char*)(&(x)))[1]

boolean LeftButton = false;         // マウス左ボタン
boolean MidButton = false;          // マウス真ん中ボタン
boolean RightButton = false;        // マウス右ボタン
int16_t dx;                            // マウスX軸
int16_t dy;                            // マウスY軸
uint8_t MSCTRL;
uint8_t oldCTRL;
byte MSDATA;

volatile uint8_t usb_keycode = 0x00;
volatile uint8_t x68_scancode = 0x00;
volatile uint8_t config_mode = 0;
volatile uint8_t state = 0;
volatile uint8_t sel_layer = 0;

volatile uint8_t prev_key = 0x00;
volatile uint8_t oneshot_sc = 0x00;
volatile uint8_t one_count = 0;


// キーリピートの定義
volatile uint8_t repeat_delay[16] = { 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170 };
volatile uint8_t repeat_rate[16] = { 3, 4, 5, 8, 11, 16, 21, 28, 35, 44, 53, 64, 75, 87, 101, 116 };

volatile uint8_t int_count = 0;

volatile uint8_t delay_time = 50;
volatile uint8_t repeat_time = 11;
volatile uint8_t need_send = 0;

volatile uint8_t data_1 = 0x00;
volatile uint8_t data_2 = 0x00;

uint8_t classType = 0;      
uint8_t subClassType = 0;


//-----------------------------------------------------------------------------

//
// HIDキーボード レポートパーサークラスの定義
//
class KbdRptParser : public KeyboardReportParser {
  protected:
    virtual uint8_t HandleLockingKeys(USBHID *hid, uint8_t key);
    virtual void OnControlKeysChanged(uint8_t before, uint8_t after);
    virtual void OnKeyDown(uint8_t mod, uint8_t key);
    virtual void OnKeyUp(uint8_t mod, uint8_t key);
    virtual void OnKeyPressed(uint8_t key) {};
};

//
// HIDマウス レポートパーサークラスの定義
//
class MouseRptParser : public MouseReportParser {
  protected:
    void OnMouseMove  (MOUSEINFO *mi);
    void OnLeftButtonUp (MOUSEINFO *mi);
    void OnLeftButtonDown (MOUSEINFO *mi);
    void OnRightButtonUp  (MOUSEINFO *mi);
    void OnRightButtonDown  (MOUSEINFO *mi);
    void OnMiddleButtonUp (MOUSEINFO *mi);
    void OnMiddleButtonDown (MOUSEINFO *mi);
};

//-----------------------------------------------------------------------------

USB     Usb;
USBHub  Hub(&Usb);

//BTD     Btd(&Usb);
//BTHID   bthid(&Btd);

//BTHID   bthid(&Btd, PAIR);
//BTHID   bthid(&Btd, PAIR, "0000");

HIDBoot<3>    HidComposite(&Usb);
HIDBoot<1>    HidKeyboard(&Usb);
HIDBoot<2>    HidMouse(&Usb);
//HIDBoot<USB_HID_PROTOCOL_KEYBOARD | USB_HID_PROTOCOL_MOUSE> HidComposite(&Usb);
//HIDBoot<USB_HID_PROTOCOL_KEYBOARD>    HidKeyboard(&Usb);
//HIDBoot<USB_HID_PROTOCOL_MOUSE>    HidMouse(&Usb);

KbdRptParser keyboardPrs;
MouseRptParser MousePrs;

//-----------------------------------------------------------------------------

void MouseRptParser::OnMouseMove(MOUSEINFO *mi) {

  // 前回のレポートからの差分が通知されてくる    
  dx += mi->dX;
  if (dx > 255) dx = 255;
  if (dx < -255) dx = -255;
  
  dy += mi->dY;
  if (dy > 255) dy = 255;
  if (dy < -255) dy = -255;
};

void MouseRptParser::OnLeftButtonUp (MOUSEINFO *mi) {
//    Serial.println("L Butt Up");
    LeftButton = false;
};
void MouseRptParser::OnLeftButtonDown (MOUSEINFO *mi) {
//    Serial.println("L Butt Dn");
    LeftButton = true;
};
void MouseRptParser::OnRightButtonUp  (MOUSEINFO *mi) {
//    Serial.println("R Butt Up");
    RightButton = false;
};
void MouseRptParser::OnRightButtonDown  (MOUSEINFO *mi) {
//    Serial.println("R Butt Dn");
    RightButton = true;
};
void MouseRptParser::OnMiddleButtonUp (MOUSEINFO *mi) {
//    Serial.println("M Butt Up");
    MidButton = false;
};
void MouseRptParser::OnMiddleButtonDown (MOUSEINFO *mi) {
//    Serial.println("M Butt Dn");
    MidButton = true;
};

//--------------------------------------------------------------------------


//
// キーが押された
//
void send_make(uint8_t key) {

  prev_key = key;

  if (key == EEPROM.read(1)) {
    sel_layer = 1;
    return;
  } else if (key == EEPROM.read(2)) {
    return;
  }

  if (sel_layer == 0) {
    x68_scancode = EEPROM.read(key);
  } else {
    x68_scancode = EEPROM.read(key + 256);
  }
  
  Serial.write(x68_scancode);
  int_count = delay_time;
}

//
// キーが開放された
//
void send_break(uint8_t key) {

  if (key == EEPROM.read(1)) {
    sel_layer = 0;

    // 
    if (prev_key == key){
      // 直前に押されたキーが自分自身の場合はまずmakeを即時で送信
      uint8_t _sc = EEPROM.read(key);
      Serial.write(_sc);

      // 30ms後にbreakも飛ぶように設定
      one_count = 3;
      oneshot_sc = _sc | 0x80;
      x68_scancode = 0;
    }
    
    return;
  } else if (key == EEPROM.read(2)) {
    sel_layer = (sel_layer == 0 ? 1 : 0);
    return;
  }
  
  if (sel_layer == 0) {
    x68_scancode = EEPROM.read(key);
  } else {
    x68_scancode = EEPROM.read(key + 256);
  }

  Serial.write(x68_scancode | 0x80);
  x68_scancode = 0;
}


// タイマー割り込み処理
void sendRepeat() {
  if (int_count > 0) int_count--;
  if (one_count > 0) one_count--;
}

  
//
// ロックキー（NumLock/CAPSLock/ScrollLock)ハンドラ
//
uint8_t KbdRptParser::HandleLockingKeys(USBHID *hid, uint8_t key) {
  if (classType == USB_CLASS_WIRELESS_CTRL) {
    uint8_t old_keys = kbdLockingKeys.bLeds;  
    switch (key) {
      case UHS_HID_BOOT_KEY_CAPS_LOCK:
        kbdLockingKeys.kbdLeds.bmCapsLock = ~kbdLockingKeys.kbdLeds.bmCapsLock;
        break;
    }
    
  } else {
    return KeyboardReportParser::HandleLockingKeys(hid, key);   
  }
  return 0;
}


//
// キー押しハンドラ
// 引数
//  mod : コントロールキー状態
//  key : HID Usage ID 
//
void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key) {
  send_make(key);
}

//
// キー離し ハンドラ
// 引数
//  mod : コントロールキー状態
//  key : HID Usage ID 
//
void KbdRptParser::OnKeyUp(uint8_t mod, uint8_t key) {
    send_break(key);
}

//
// コントロールキー変更ハンドラ
// SHIFT、CTRL、ALT、GUI(Win)キーの処理を行う
// 引数 before : 変化前のコード USB Keyboard Reportの1バイト目
//      after  : 変化後のコード USB Keyboard Reportの1バイト目
//
void KbdRptParser::OnControlKeysChanged(uint8_t before, uint8_t after) {
  MODIFIERKEYS beforeMod;
  *((uint8_t*)&beforeMod) = before;

  MODIFIERKEYS afterMod;
  *((uint8_t*)&afterMod) = after;

  uint8_t code;

  // 左Ctrlキー
  if (beforeMod.bmLeftCtrl != afterMod.bmLeftCtrl) {
    if (afterMod.bmLeftCtrl) {
      send_make(0xE0);      // 左Ctrlキーを押した
    } else {
      send_break(0xE0);      // 左Ctrlキーを離した
    } 
  }

  // 左Shiftキー
  if (beforeMod.bmLeftShift != afterMod.bmLeftShift) {
    if (afterMod.bmLeftShift) {
      send_make(0xE1);      // 左Shiftキーを押した
    } else {
      send_break(0xE1);      // 左Shiftキーを離した
    }
  }

  // 左Altキー(XF1)
  if (beforeMod.bmLeftAlt != afterMod.bmLeftAlt) {
    if (afterMod.bmLeftAlt) {
      send_make(0xE2);      // 左Altキーを押した
    } else {
      send_break(0xE2);      // 左Altキーを離した
    }
  }

  // 左GUIキー(Winキー)(ひらがな)
  if (beforeMod.bmLeftGUI != afterMod.bmLeftGUI) {
    if (afterMod.bmLeftGUI) {
      send_make(0xE3);      // 左Altキーを押した
    } else {
      send_break(0xE3);      // 左Altキーを離した
    }
  }

  // 右Ctrlキー(OPT.2)
  if (beforeMod.bmRightCtrl != afterMod.bmRightCtrl) {
    if (afterMod.bmRightCtrl) {
      send_make(0xE4);      // 左Altキーを押した
    } else {
      send_break(0xE4);      // 左Altキーを離した
    }
  }

  // 右Shiftキー
  if (beforeMod.bmRightShift != afterMod.bmRightShift) {
    if (afterMod.bmRightShift) {
      send_make(0xE5);      // 左Altキーを押した
    } else {
      send_break(0xE5);      // 左Altキーを離した
    }
  }

  // 右Altキー(XF5)
  if (beforeMod.bmRightAlt != afterMod.bmRightAlt) {
    if (afterMod.bmRightAlt) {
      send_make(0xE6);      // 左Altキーを押した
    } else {
      send_break(0xE6);      // 左Altキーを離した
    }
  }

  // 右GUIキー(opt1)
  if (beforeMod.bmRightGUI != afterMod.bmRightGUI) {
    if (afterMod.bmRightGUI) {
      send_make(0xE7);      // 左Altキーを押した
    } else {
      send_break(0xE7);      // 左Altキーを離した
    }
  }
}

//
// インターフェースクラスの取得
//
uint8_t getIntClass(byte& intclass, byte& intSubClass ) {
  uint8_t buf[ 256 ];
  uint8_t* buf_ptr = buf;
  byte rcode;
  byte descr_length;
  byte descr_type;
  unsigned int total_length;

  uint8_t flgFound = 0;
  
  //デスクプリタトータルサイズの取得
  rcode = Usb.getConfDescr( 1, 0, 4, 0, buf );
  LOBYTE( total_length ) = buf[ 2 ]; HIBYTE( total_length ) = buf[ 3 ];
  if ( total_length > 256 ) {
    total_length = 256;
  }
  
  rcode = Usb.getConfDescr( 1, 0, total_length, 0, buf ); 
  while ( buf_ptr < buf + total_length ) { 
    descr_length = *( buf_ptr );
    descr_type = *( buf_ptr + 1 );

    if (descr_type == USB_DESCRIPTOR_INTERFACE) {
      // 最初のインタフェースの取得
      USB_INTERFACE_DESCRIPTOR* intf_ptr = ( USB_INTERFACE_DESCRIPTOR* )buf_ptr;  
      intclass = intf_ptr->bInterfaceClass;
      intSubClass = intf_ptr->bInterfaceSubClass;
      flgFound = 1;
      break;
    }
    buf_ptr = ( buf_ptr + descr_length );    //advance buffer pointer
  }
  return ( flgFound );
}

//
// マウスデータ送信部分
//
void mouse_send() {

    MSDATA = 0;
    if (LeftButton) MSDATA |= B00000001;    // 左クリック
    if (RightButton) MSDATA |= B00000010;   // 右クリック

    MSSerial.write(MSDATA);
    delayMicroseconds(250);
    
    MSSerial.write((int8_t)(dx/2));
    delayMicroseconds(250);
    dx = 0; // 一度送信したらリセット
 
    MSSerial.write((int8_t)(dy/2));
    delayMicroseconds(250);
    dy = 0; // 一度送信したらリセット

}


void setup() {

  MSSerial.begin(4800);

#if MYDEBUG
  Serial.println("Self Test OK.");
#endif

// USB初期化
  if (Usb.Init() == -1) {
#if MYDEBUG
    Serial.println(F("OSC did not start."));
#endif
    while (1); // Halt    
  }
  delay( 200 );

  Serial.begin(2400);  //MSDATA送信用


#if MYDEBUG
  Serial.println(F("HID Start."));
#endif

  HidComposite.SetReportParser(0, &keyboardPrs);
  HidComposite.SetReportParser(1, &MousePrs);
  HidKeyboard.SetReportParser(0, &keyboardPrs);
  HidMouse.SetReportParser(0, &MousePrs);

  config_mode = 0;

  MsTimer2::set(10, sendRepeat); 
  MsTimer2::start();

}

void config_map(uint8_t data) {

  uint8_t ret, _layer, _sc, j;

  switch(state) {
    case 0:
      state = 1;
      data_1 = data;
      Serial.write(RET_D1_OK);
      break;
    default:
      state = 0;
      data_2 = data;
      
      if (data_1 <= 0x03) {
        EEPROM.write(data_1, data_2);
        Serial.write(RET_D2S_OK);
      } else {
        if ((data_1 == 0x7F) && (data_2 == 0x7F)) {
          // 7F7Fは終了コマンド
          Serial.write(RET_OK_EXIT);
          config_mode = 0;
        } else {
          _layer = (data_2 & 0x80);
          _sc = (data_2 & 0x7F);
          if (_layer == 0) {
            // Layer0
            EEPROM.write(data_1, _sc);
            Serial.write(RET_D2A_OK);
          } else {
            // Layer1
            EEPROM.write(data_1 + 256, _sc);
            Serial.write(RET_D2B_OK);
          }
        }
      }
      break;
  }
}


void loop() {

  Usb.Task();

  //KBDSerial.listen();
  if (Serial.available()) {  //データなしは-1が流れてる

    MSCTRL = Serial.read();

    if (config_mode != 0) {
      config_map(MSCTRL);
      return;
    }

    switch (MSCTRL >> 6) {
      case B00:
        // TV Control
        if (MSCTRL == 0x0D) {
          state = 0;
          config_mode = 1;
          Serial.write(RET_OK);
        }
        break;
      case B01:
        //
        if ((MSCTRL >> 4) == B0110) {
          // Delay time
          delay_time = repeat_delay[MSCTRL & 0x0F];
        }
        if ((MSCTRL >> 4) == B0111) {
          // Repeat time
          repeat_time = repeat_rate[MSCTRL & 0x0F];
        } 
        if ((MSCTRL >> 3) == B01000) {
          // MS Control H -> L
          if ((MSCTRL & 1) == 0) mouse_send();

          // 流れてくるデータを確認する限り、不定bitは必ず0、つまり
          // 0x40(H->L)か0x41(L->H)しか送信されてこないような気がする
        }
        break;
      case B10:
      case B11:
        // LED Control
        break;
    }
  }

  if (int_count == 0) {
    if (x68_scancode != 0x00) Serial.write(x68_scancode);
    int_count = repeat_time;
  }

  if (one_count == 0) {
    if (oneshot_sc != 0x00) Serial.write(oneshot_sc);
  }
  
}
