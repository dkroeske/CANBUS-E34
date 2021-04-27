#include <SPI.h>

/* 
 * function prototypes
 */
void can_init();
uint8_t mcp2515_read(uint8_t addr);

/*
 * Pinning
 */
#define SPI_CS 9

// Setup ones
void setup() {

  //
  can_init();

}

// Loop forever
void loop() {
  // put your main code here, to run repeatedly:
  delay(100);

}

/* *************************************************** */
void can_init(){
  // Serial port
  Serial.begin(115200);
  
  //
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  // Setup SPI: CS=#D9, MOSI=#D11, MISO=#D12, SCL=#D13
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  
  // Reset MCP2515 chip
  mcp2515_reset();

  delay(1000);

  uint8_t data = mcp2515_read(0x0F);

  // Formatted print Arduino style
  char buf[64];
  sprintf(buf, "CANCTRL: 0x%.2X\n", data);
  Serial.print(buf);
  
  // Set mode (NORMAL of LOOPBACK)
}

/* *************************************************** */
void mcp2515_reset() {
  digitalWrite(SPI_CS, LOW);
  SPI.transfer(0xC0);
  digitalWrite(SPI_CS, HIGH);
}

/* *************************************************** */
uint8_t mcp2515_read(uint8_t addr) {

  // Setup tx buffer
  uint8_t txbuf[2];
  txbuf[0] = 0x03;
  txbuf[1] = addr;

  // Tx to mcp2515
  digitalWrite(SPI_CS, LOW);
  SPI.transfer(txbuf, 2);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(SPI_CS, HIGH);
  
  // Return
  return value;
}
