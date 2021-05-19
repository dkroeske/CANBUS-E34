#include <SPI.h>
#include "mcp2515_regs.h"



/* 
 * function prototypes
 */

/*
 * CAN function prototypes
 */
void can_init();
uint8_t can_tx_extended_data_frame(uint32_t id, uint8_t *data);

/*
 * mcp2515 function prototypes
 */
void mcp2515_init(void);
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

/*
 * Global var's
 */
char debug_buf[64];

// Setup runs one time
void setup() {
  Serial.begin(115200);
  can_init();
}

// Loop forever
void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}


/*
 * 
 * CAN init, RX and TX functions
 * 
 */


/* ************************************************************** */
void can_init(void)
/* 
short  :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
  Serial.print(">> can_init\n");

  // Setup SPI
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  // Setup SPI: CS=#D9, MOSI=#D11, MISO=#D12, SCL=#D13
  SPI.begin();
  // Opt. slow down SPI (bij lange draadjes)
  // SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  // Setup interrupt handler for mcp2515 interrupts
  //gpio_set_irq_enabled_with_callback(21, GPIO_IRQ_EDGE_FALL, true, &mcp2515_callback);

  // Reset and init mcp2515
  mcp2515_init();

  Serial.print("<< can_init\n");
}

///**************************************************************** */
//uint8_t can_read_msg(CAN_DATA_FRAME_STRUCT *frame) 
///* 
//short  :         
//inputs  :        
//outputs : 
//notes :         
//Version : DMK, Initial code
//*******************************************************************/
//{
//    uint8_t buf[14];
//   
//    // Read queue
//    bool done = queue_try_remove(&rx_message_queue, buf);
//    
//    if(done) {
//        // Check if standard or extended id 
//        if( buf[1] & 0x08 ) {
//            // Extended ID
//            uint8_t b3 = (buf[0] >> 3);  
//            uint8_t b2 = (buf[0] << 5) | ((buf[1]&0xE0) >>3) | (buf[1]&0x03);
//            uint8_t b1 = buf[2];
//            uint8_t b0 = buf[3];
//            frame->id = b3 << 24 | b2 << 16 | b1 << 8 | b0;
//        } else {
//            // Standard ID
//            frame->id = (buf[1] << 3) | (buf[1] >> 5);
//        }
//        
//        // Get datalenght (clip to max. 8 bytes) 
//        frame->datalen = (buf[4]&0x0F) <= 8 ? buf[4]&0x0F : 8;
//        
//        // Get actual data
//        for(uint8_t idx = 0; idx < frame->datalen; idx++) {
//           frame->data[idx] = buf[5+idx];
//        }
//    }
//    return done;
//}
//

/**************************************************************** */
uint8_t can_tx_extended_data_frame(uint32_t id, uint8_t *data, uint8_t nr_bytes)
/* 
short :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
*******************************************************************/
{
    uint8_t err = 0, tx_buf_id;

    // Find free transmitbuffer (out of 3)
    uint8_t status = mcp2515_read_status();
    if( !(status & 0x04) ) {
        tx_buf_id = 0x00;
    } else if ( !(status & 0x10) ) {
      tx_buf_id = 0x01;
    } else if ( !(status & 0x40) ) {
      tx_buf_id = 0x02;
    } else {
      err = 1; // No free transmit slot 
    }

    /* Display empty buffer id */
    sprintf(debug_buf,"\Free tx_buf_id: 0x%.2X\n", tx_buf_id);
    Serial.print(debug_buf);

    // If free transmitterbuffer  
    if( 0 == err ) {
    
        // Temp transmitbuffer and make sure lenght <= 8
        uint8_t buf[14];
        uint8_t datalen = nr_bytes <= 8 ? nr_bytes : 8;

        // Construct buffer content for extended id
        buf[0] = (uint8_t) ((id << 3) >> 24) ;    // TXBnSIDH
        buf[1] = (uint8_t) ((((id << 3) | (id & 0x00030000)) >> 16) | EXIDE); // TXBnSIDL
        buf[2] = id >> 8; // TXBnEID8      
        buf[3] = id;      // TXBnEID0
        buf[4] = datalen; // TXBnDLC and RTR bit clear
    
        // TBnDm registers
        for(uint8_t idx = 0; idx < datalen; idx++) {
            buf[5+idx] = data[idx];
        }
    
        // Write to CAN controller
        mcp2515_load_tx_buffer(tx_buf_id, buf, 13);
    
        // ... and request transmit
        mcp2515_RTS(tx_buf_id);
    }

    return err;
}


/* ************************************************************** */
void mcp2515_init() 
/* 
short  :         
inputs  :        
outputs : 
notes :         
Version : DMK, Initial code
***************************************************************** */
{
  
    // Reset CAN controller
    mcp2515_reset();
    delay(100);

    // Clear masks RX messages
    mcp2515_write_register(RXM0SIDH, 0x00);
    mcp2515_write_register(RXM0SIDL, 0x00);
    mcp2515_write_register(RXM0EID8, 0x00);
    mcp2515_write_register(RXM0EID0, 0x00);

    // Clear filter and EXIDE bit
    mcp2515_write_register(RXF0SIDL, 0x00);

    // Set CNF1 (250kbps) Berekent met de Microchip calculator
    mcp2515_write_register(CNF1, 0x00); // 0x00
  
    // Set CNF2 (
    mcp2515_write_register(CNF2, 0xB1); // 0xB1
  
    // Set CNF3 (
    mcp2515_write_register(CNF3, 0x85); // 0x85

    // Interrupts on rx buffers and errors
    //mcp2515_write_register(CANINTE, MERRE | ERRIE | RX1IE | RX0IE); 
    mcp2515_write_register(CANINTE, 0x00); 

    // Set NORMAL mode (or LOOPBACK for development)
    //uint8_t mode = REQOP_LOOPBACK; 
    uint8_t mode = REQOP_NORMAL;

    mcp2515_write_register(CANCTRL, mode);

    // Verify mode
    uint8_t dummy = mcp2515_read_register(CANSTAT);
    if (mode != (dummy && 0xE0)) {
        Serial.println("Error setting mode!");
        mcp2515_write_register(CANCTRL, mode);
    }
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
        sprintf(debug_buf, "\Invalid tx_buf_id: 0x%.2X\n", tx_buf_id);
        Serial.print(debug_buf);
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
