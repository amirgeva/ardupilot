/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#include <ToneAlarm_PX4.h>

extern ToneAlarm_PX4 tonealarm;

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
  static const uint32_t CMD_RC   = 0x00000001;
  static const uint32_t CMD_CFG  = 0x00000002;
  static const uint32_t CMD_TONE = 0x00000003;
  
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
  static const uint8_t REQ_OPT_VX    = 0x10;
  static const uint8_t REQ_OPT_VY    = 0x11;
  static const uint8_t REQ_OPT_LAST  = 0x12;
  
  
  // Maximum number of telemetry data fields
  enum { REQUEST_LIMIT = 32 };
  
  AP_HAL::UARTDriver* m_UART;
  int                 m_Frame;
  uint16_t            m_OverrideCount;
  uint8_t             m_Requests[REQUEST_LIMIT];
  uint32_t            m_BufferSize;
  
  void Log(const char* s)
  {
    //gcs_send_text_P(SEVERITY_LOW, PSTR(s));
  }
  
  struct CMD_RCOverride
  {
    int16_t  chans[8];
    void init() {}
  };
  
  struct CMD_Config
  {
    uint32_t buffer_size;
    uint8_t  requests[REQUEST_LIMIT];
    void init() 
    { 
      buffer_size=64;
      memset(requests,0,sizeof(requests)); 
    }
  };
  
  struct CMD_Tone
  {
    uint8_t playstr[54];
    void init()
    {
      memset(playstr,0,sizeof(playstr));
    }
  };
  
  class Buffer
  {
  public:
    Buffer(unsigned sz)
    : m_Size(sz)
    , m_Buffer(new uint8_t[sz])
    {
      init();
    }
    
    ~Buffer()
    {
      delete[] m_Buffer;
    }

    Buffer(const Buffer& rhs) 
    : m_Size(rhs.m_Size)
    , m_Buffer(new uint8_t[rhs.m_Size])
    {
      memcpy(m_Buffer,rhs.m_Buffer,m_Size);
    }
    
    Buffer& operator= (const Buffer& rhs) 
    {
      // Allocate first to temporary (exception safety)
      uint8_t* newbuf=new uint8_t[rhs.m_Size];
      delete[] m_Buffer;
      m_Size=rhs.m_Size;
      m_Buffer=newbuf;
      memcpy(m_Buffer,rhs.m_Buffer,m_Size);
      return *this; 
    }
    
    unsigned size() const { return m_Size; }
    
    void init()
    {
      memset(m_Buffer,0,m_Size);
      *(reinterpret_cast<uint32_t*>(m_Buffer)) = SYNC_FLAG;
    }
    
    void send(AP_HAL::UARTDriver* uart)
    {
      uart->write(m_Buffer,m_Size);
    }
    
    void calc_crc()
    {
      uint16_t crc=0xFFFF;
      int n=m_Size-sizeof(uint16_t);
      for(int i=0;i<n;++i)
        crc=crc16(crc,m_Buffer[i]);
      *((uint16_t*)(m_Buffer+n)) = crc;
    }
    
    uint16_t get_crc() const
    {
      return *(reinterpret_cast<const uint16_t*>(m_Buffer+(m_Size-2)));
    }
    
    uint8_t* ptr() { return m_Buffer; }
  private:
    unsigned m_Size;
    uint8_t* m_Buffer;
  };
  
  void read_command_details()
  {
    Buffer buf(64);
    uint8_t* ptr = buf.ptr();
    uint32_t* header=reinterpret_cast<uint32_t*>(ptr);
    uint8_t* payload=(ptr+2*sizeof(uint32_t));
    *ptr = m_UART->read();
    if (*ptr!=0xEF) return;
    for(int i=0;i<3;++i)
      *(++ptr)=m_UART->read();
    if (header[0] != SYNC_FLAG) return;
    for(int i=0;i<60;++i)
      *(++ptr)=m_UART->read();
    uint16_t recv_crc=buf.get_crc();
    buf.calc_crc();
    if (recv_crc != buf.get_crc()) return;
    if (header[1] == CMD_RC)
    {
      CMD_RCOverride* rc=reinterpret_cast<CMD_RCOverride*>(payload);
      hal.rcin->set_overrides(rc->chans,8);
      m_OverrideCount=0;
      copter.failsafe.rc_override_active = true;
      copter.failsafe.last_heartbeat_ms = hal.scheduler->millis();
    }
    if (header[1] == CMD_CFG)
    {
      CMD_Config* cmd=reinterpret_cast<CMD_Config*>(payload);
      m_BufferSize=cmd->buffer_size;
      for(int i=0;i<REQUEST_LIMIT;++i) 
        m_Requests[i]=cmd->requests[i];
    }
    if (header[1] == CMD_TONE)
    {
      CMD_Tone* cmd=reinterpret_cast<CMD_Tone*>(payload);
      tonealarm.play_string((const char*)cmd->playstr);
    }
  }
  
  float get_opt_vx()
  {
    if (!copter.optflow.enabled()) return -1;
    return copter.optflow.bodyRate().x;
  }

  float get_opt_vy()
  {
    if (!copter.optflow.enabled()) return -1;
    return copter.optflow.bodyRate().y;
  }

public:
  ISL_Serial_Protocol()
  : m_UART(0)
  , m_Frame(0)
  , m_OverrideCount(0)
  , m_BufferSize(64)
  {
  }
  
  void init()
  {
    Log("Initializing ISL Serial");
    m_UART = copter.serial_manager.find_serial(AP_SerialManager::SerialProtocol_ISL,0);
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
    if (++m_OverrideCount > 100) // Timeout receiving override command (1 second).  Revert to manual
    {
      m_OverrideCount=100;
      copter.failsafe.rc_override_active = false;
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
  
  #define GPS_FILTER(x) (copter.gps.status()>AP_GPS::GPS_OK_FIX_2D?x:0)
  
  void write_telemetry()
  {
    Buffer tb(m_BufferSize);
    int bytes_left=tb.size() - sizeof(uint32_t) - sizeof(uint16_t);
    uint8_t* ptr=tb.ptr()+sizeof(uint32_t);
    for(int i=0;i<REQUEST_LIMIT;++i)
    {
      switch (m_Requests[i])
      {
        case REQ_NONE:  break;
        case REQ_PITCH: ADD_FIELD(float,copter.ahrs.pitch);
        case REQ_ROLL:  ADD_FIELD(float,copter.ahrs.roll);
        case REQ_YAW:   ADD_FIELD(float,copter.ahrs.yaw);
        case REQ_ALT:   ADD_FIELD(float,copter.barometer.get_altitude());
        case REQ_RC1:   ADD_FIELD(uint16_t,hal.rcin->read(0));
        case REQ_RC2:   ADD_FIELD(uint16_t,hal.rcin->read(1));
        case REQ_RC3:   ADD_FIELD(uint16_t,hal.rcin->read(2));
        case REQ_RC4:   ADD_FIELD(uint16_t,hal.rcin->read(3));
        case REQ_RC5:   ADD_FIELD(uint16_t,hal.rcin->read(4));
        case REQ_RC6:   ADD_FIELD(uint16_t,hal.rcin->read(5));
        case REQ_RC7:   ADD_FIELD(uint16_t,hal.rcin->read(6));
        case REQ_RC8:   ADD_FIELD(uint16_t,hal.rcin->read(7));
        case REQ_GPS_LNG:   ADD_FIELD(int32_t,GPS_FILTER(copter.gps.location().lng));
        case REQ_GPS_LAT:   ADD_FIELD(int32_t,GPS_FILTER(copter.gps.location().lat));
        case REQ_GPS_ALT:   ADD_FIELD(int32_t,GPS_FILTER(copter.gps.location().alt));
        case REQ_OPT_VX:  ADD_FIELD(float,get_opt_vx());
        case REQ_OPT_VY:  ADD_FIELD(float,get_opt_vy());
        case REQ_OPT_LAST: ADD_FIELD(uint32_t,copter.optflow.last_update());
      }
    }
    tb.calc_crc();
    tb.send(m_UART);
  }
;  
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
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    g_ISL_Serial_Protocol.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    g_ISL_Serial_Protocol.fast_loop();
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    g_ISL_Serial_Protocol.slow_loop();
}
#endif
