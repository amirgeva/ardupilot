/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

inline uint16_t crc16(uint16_t crc, uint8_t b)
{
  crc ^= b;
  for (int i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}



class ISL_Serial_Protocol
{
  static const uint32_t SYNC_FLAG = 0xFEEDBEEF;
  static const uint32_t CMD_RC  = 0x00000001;
  static const uint32_t CMD_CFG = 0x00000002;
  
  static const uint8_t REQ_NONE      = 0x00;
  static const uint8_t REQ_PITCH     = 0x01;
  static const uint8_t REQ_ROLL      = 0x02; 
  static const uint8_t REQ_YAW       = 0x03;  
  static const uint8_t REQ_ALT       = 0x04;  
  static const uint8_t REQ_RC1       = 0x05;  
  static const uint8_t REQ_RC2       = 0x06;  
  static const uint8_t REQ_RC3       = 0x07;  
  static const uint8_t REQ_RC4       = 0x08;  
  static const uint8_t REQ_RC5       = 0x09;  
  static const uint8_t REQ_RC6       = 0x0a;  
  static const uint8_t REQ_RC7       = 0x0b;  
  static const uint8_t REQ_RC8       = 0x0c;    
  static const uint8_t REQ_GPS_LNG   = 0x0d;    
  static const uint8_t REQ_GPS_LAT   = 0x0e;    
  static const uint8_t REQ_GPS_ALT   = 0x0f;    
  
  AP_HAL::UARTDriver* m_UART;
  int                 m_Frame;
  uint16_t            m_OverrideCount;
  uint8_t             m_Requests[16];
  
  void Log(const char* s)
  {
    gcs_send_text_P(SEVERITY_LOW, PSTR(s));
  }
  
  struct Telemetry
  {
    unsigned int sync_flag;    //  4
    uint8_t data[58];          // 58 
    uint16_t crc;              //  2
    // Total                      64
    
    void init()
    {
      sync_flag = SYNC_FLAG;
    }
  };
  
  struct CMD_RCOverride
  {
    uint32_t sync_flag;
    uint32_t command;
    int16_t  chans[8];
    void init() {}
  };
  
  struct CMD_Config
  {
    uint32_t sync_flag;
    uint32_t command;
    uint8_t requests[16];
    void init() { memset(requests,0,sizeof(requests)); }
  };
  
  class Buffer
  {
  public:
    void init()
    {
      memset(buffer,0,64);
    }
    void send(AP_HAL::UARTDriver* uart)
    {
      uart->write(buffer,BUFFER_SIZE);
    }
    
    void calc_crc()
    {
      uint16_t crc=0xFFFF;
      int n=BUFFER_SIZE-sizeof(uint16_t);
      for(int i=0;i<n;++i)
        crc=crc16(crc,buffer[i]);
      *((uint16_t*)(buffer+n)) = crc;
    }
    
    uint16_t get_crc() const
    {
      return *(reinterpret_cast<const uint16_t*>(buffer+BUFFER_SIZE-2));
    }
    
    uint8_t* ptr() { return buffer; }
  private:
    enum { BUFFER_SIZE = 64 };
    uint8_t buffer[BUFFER_SIZE];
  };
  
  struct TelemetryBuffer
  {
    TelemetryBuffer() 
    { 
      data.b.init();
      data.t.init();
    }
    union {
      Telemetry t;
      Buffer    b;
    } data;
  };
  
  struct CommandBuffer
  {
    CommandBuffer()
    {
      data.rc.init();
      data.cfg.init();
      data.b.init();
    }
    union {
      CMD_RCOverride rc;
      CMD_Config     cfg;
      Buffer         b;
    } data;
  };
  
  void read_command_details()
  {
    CommandBuffer cmd;
    uint8_t* ptr = cmd.data.b.ptr();
    *ptr = m_UART->read();
    if (*ptr!=0xEF) return;
    for(int i=0;i<3;++i)
      *(++ptr)=m_UART->read();
    if (cmd.data.rc.sync_flag != SYNC_FLAG) return;
    for(int i=0;i<60;++i)
      *(++ptr)=m_UART->read();
    uint16_t recv_crc=cmd.data.b.get_crc();
    cmd.data.b.calc_crc();
    if (recv_crc != cmd.data.b.get_crc()) return;
    if (cmd.data.rc.command == CMD_RC)
    {
      hal.rcin->set_overrides(cmd.data.rc.chans,8);
      m_OverrideCount=0;
      failsafe.rc_override_active = true;
      failsafe.last_heartbeat_ms = millis();
    }
    if (cmd.data.rc.command == CMD_CFG)
    {
      for(int i=0;i<16;++i) 
        m_Requests[i]=cmd.data.cfg.requests[i];
    }
  } 

public:
  ISL_Serial_Protocol()
  : m_UART(0)
  , m_Frame(0)
  , m_OverrideCount(0)
  {
  }
  
  void init()
  {
    Log("Initializing ISL Serial");
    m_UART = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ISL,0);
    if (!m_UART)
    {
      Log("Failed to initialize ISL Serial");
    }
  }
  
  void read_commands()
  {
    if (m_UART->available() >= 64)
    {
      read_command_details();
    }
    if (++m_OverrideCount > 100) // Timeout receiving override command.  Revert to manual
    {
      m_OverrideCount=100;
      failsafe.rc_override_active = false;
      hal.rcin->clear_overrides();
    }
  }
  
  #define ADD_FIELD(type,value) {\
    int sz=sizeof(type);\
    if (sz<=bytes_left) {\
      type v=value;\
      uint8_t* vptr=reinterpret_cast<uint8_t*>(&v);\
      memcpy(ptr,vptr,sz);\
      ptr+=sz;\
      bytes_left-=sz;\
    }\
  } break
  
  #define GPS_FILTER(x) (gps.status()>AP_GPS::GPS_OK_FIX_2D?x:0)
  
  void write_telemetry()
  {
    TelemetryBuffer tb;
    int bytes_left=sizeof(tb.data.t.data);
    uint8_t* ptr=tb.data.b.ptr()+4;
    for(int i=0;i<16;++i)
    {
      switch (m_Requests[i])
      {
        case REQ_NONE:  break;
        case REQ_PITCH: ADD_FIELD(float,ahrs.pitch);
        case REQ_ROLL:  ADD_FIELD(float,ahrs.roll);
        case REQ_YAW:   ADD_FIELD(float,ahrs.yaw);
        case REQ_ALT:   ADD_FIELD(float,barometer.get_altitude());
        case REQ_RC1:   ADD_FIELD(uint16_t,hal.rcin->read(0));
        case REQ_RC2:   ADD_FIELD(uint16_t,hal.rcin->read(1));
        case REQ_RC3:   ADD_FIELD(uint16_t,hal.rcin->read(2));
        case REQ_RC4:   ADD_FIELD(uint16_t,hal.rcin->read(3));
        case REQ_RC5:   ADD_FIELD(uint16_t,hal.rcin->read(4));
        case REQ_RC6:   ADD_FIELD(uint16_t,hal.rcin->read(5));
        case REQ_RC7:   ADD_FIELD(uint16_t,hal.rcin->read(6));
        case REQ_RC8:   ADD_FIELD(uint16_t,hal.rcin->read(7));
        case REQ_GPS_LNG:   ADD_FIELD(int32_t,GPS_FILTER(gps.location().lng));
        case REQ_GPS_LAT:   ADD_FIELD(int32_t,GPS_FILTER(gps.location().lat));
        case REQ_GPS_ALT:   ADD_FIELD(int32_t,GPS_FILTER(gps.location().alt));
      }
    }
    tb.data.b.calc_crc();
    tb.data.b.send(m_UART);
  }
  
  void fast_loop()
  {
    if (m_UART)
    {
      read_commands();
      write_telemetry();
    }
  }
  
  void slow_loop()
  {
    if (m_UART) Log("UART init ok.");
    else Log("UART not found.");
  }
} g_ISL_Serial_Protocol;


#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    g_ISL_Serial_Protocol.init();
}
#endif

#ifdef USERHOOK_FASTLOOP

void userhook_FastLoop()
{
    // put your 100Hz code here
    g_ISL_Serial_Protocol.fast_loop();    
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    g_ISL_Serial_Protocol.slow_loop();    
}
#endif