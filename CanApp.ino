#include <SPI.h>

/* 
 * function prototypes
 */
void can_init();

/*
 * mcp2515 function prototypes
 */
uint8_t mcp2515_read_register(uint8_t addr);
void mcp2515_read_rx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len);
void mcp2515_write_register(uint8_t addr, uint8_t data);
void mcp2515_load_tx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len);
void mcp2515_RTS(uint8_t tx_buf_id);
uint8_t mcp2515_read_status(void);
void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t data);

/*
 * I/O Pinning
 */
#define SPI_CS 10

/* MCP2515 constants */
#define CAN_RESET       0xC0
#define CAN_READ        0x03
#define CAN_WRITE       0x02
#define CAN_RTS         0x80
#define CAN_RTS_TXB0    0x81
#define CAN_RTS_TXB1    0x82
#define CAN_RTS_TXB2    0x84
#define CAN_RD_STATUS   0xA0
#define CAN_BIT_MODIFY  0x05  
#define CAN_RX_STATUS   0xB0
#define CAN_RD_RX_BUFF  0x90
#define CAN_LOAD_TX     0X40 

#define DEBUG
      
#ifdef DEBUG
char debug_buf[64];
#endif

// Setup runs one time
void setup() {
  
  // Serial port
  Serial.begin(115200);
  
  // CAN shield
  can_init();
}


/* ************************************************************** */
void can_init()
/* 
short  :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
  // Setu[ SPI
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  // Setup SPI: CS=#D9, MOSI=#D11, MISO=#D12, SCL=#D13
  SPI.begin();
  // Opt. slow down SPI (bij lange draadjes)
  //SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  // Reset MCP2515 chip
  mcp2515_reset();
  delay(1000);

  uint8_t data = mcp2515_read_register(0x0F);

  // Formatted print Arduino style
  char buf[64];
  sprintf(buf, "CANCTRL: 0x%.2X\n", data);
  Serial.print(buf);
  
  // Set mode (NORMAL of LOOPBACK)
}


// Loop forever
void loop() {
  // put your main code here, to run repeatedly:
  delay(100);

}

/* ************************************************************** *
 *
 * All low level MCP2515 SPI commands. 
 * Chapter 12 van de datasheet
 * 
 * ************************************************************** */
 
/* ************************************************************** */
void mcp2515_reset()
/* 
short  :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
  digitalWrite(SPI_CS, LOW);
  SPI.transfer(CAN_RESET);
  digitalWrite(SPI_CS, HIGH);
}


/* ************************************************************** */
uint8_t mcp2515_read_register(uint8_t addr)
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
  uint8_t buf[2];
  buf[0] = CAN_READ;
  buf[1]= addr;
  uint8_t data;

  digitalWrite(SPI_CS, LOW);
  SPI.transfer(buf, 2);
  data = SPI.transfer(0x00);
  digitalWrite(SPI_CS, HIGH);

  return data;
}

/* *************************************************************** */
void mcp2515_read_rx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len)
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
    uint8_t cmd;
    switch(buffer_id) {
        case 0x00:
            cmd = 0x90; 
            break;
        case 0x01:
            cmd = 0x94; 
            break;
    }
    
    digitalWrite(SPI_CS, LOW);
    SPI.transfer(cmd);
    SPI.transfer(data, len);
    digitalWrite(SPI_CS, HIGH);
}

/* ************************************************************** */
void mcp2515_write_register(uint8_t addr, uint8_t data)
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
    uint8_t buf[3];
    buf[0] = CAN_WRITE;
    buf[1] = addr;
    buf[2] = data;

    digitalWrite(SPI_CS, LOW);
    SPI.transfer(buf, 3);
    digitalWrite(SPI_CS, HIGH);
} 


/* ************************************************************** */
void mcp2515_load_tx_buffer(uint8_t buffer_id, uint8_t *data, uint8_t len) 
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
    uint8_t cmd;
    switch(buffer_id) {
    case 0x00:
        cmd = 0x40;
        break;
    case 0x01:
        cmd = 0x42;
        break;
    case 0x02:
        cmd = 0x44;
        break;
    }

    digitalWrite(SPI_CS, LOW);
    SPI.transfer(cmd);
    SPI.transfer(data, len);
    digitalWrite(SPI_CS, HIGH);
}


/* ************************************************************** */
void mcp2515_RTS(uint8_t tx_buf_id)
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{

    uint8_t err = 0;
    uint8_t data = 0x00;

    switch(tx_buf_id) {
      case 0x00:
        data = CAN_RTS_TXB0;
      break;
      
      case 0x01:
        data = CAN_RTS_TXB1;
      break;
  
      case 0x02:
        data = CAN_RTS_TXB2;
      break;
  
      default:
        err = 1;
        #ifdef DEBUG
          sprintf(debug_buf, "\Invalid tx_buf_id: 0x%.2X\n", tx_buf_id);
          Serial.print(debug_buf);
        #endif
    }

    if(!err) {
      digitalWrite(SPI_CS, LOW);
      SPI.transfer(data);
      digitalWrite(SPI_CS, HIGH);
    }
}


/* ************************************************************** */
uint8_t mcp2515_read_status(void)
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{

    uint8_t txbuf[3];
    txbuf[0] = CAN_RD_STATUS;
    txbuf[1] = 0x00;
    txbuf[2] = 0x00;
    uint8_t rxbuf[2];
  
    digitalWrite(SPI_CS, LOW);
    SPI.transfer(txbuf, 3);
    SPI.transfer(rxbuf, 2);
    digitalWrite(SPI_CS, HIGH);
   
    return rxbuf[0];
}




/* ************************************************************** */
void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
    uint8_t buf[4];
    buf[0] = CAN_BIT_MODIFY;
    buf[1] = addr;
    buf[2] = mask;
    buf[2] = data;

    digitalWrite(SPI_CS, LOW);
    SPI.transfer(buf, 4);
    digitalWrite(SPI_CS, HIGH);
}
