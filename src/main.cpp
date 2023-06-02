#include <Arduino.h>
#include <mcp2515.h>
#include <driver/spi_master.h>

#define SPI_BUS HSPI_HOST    // SPI bus to use
#define PIN_SCK  14          // GPIO14 (SCK)
#define PIN_MISO 12          // GPIO12 (MISO)
#define PIN_MOSI 13          // GPIO13 (MOSI)
#define PIN_CS   15          // GPIO15 (CS)

spi_device_handle_t spi;

void spi_init() {
    esp_err_t ret;

    // SPI bus configuration
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
        
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI_BUS, &bus_config, 0);
    assert(ret == ESP_OK);

    // Device configuration
    spi_device_interface_config_t dev_config = {
        .mode = 0,                       // Set SPI data mode
        .clock_speed_hz = 1000000,       // Set SPI clock frequency (1 MHz)
        .spics_io_num = PIN_CS,          // GPIO15 (CS)
        .queue_size = 32,
    };

    // Add the SPI device to the bus
    ret = spi_bus_add_device(SPI_BUS, &dev_config, &spi);
    assert(ret == ESP_OK);
}


// put function declarations here:
int myFunction(int, int);

#define POINT_PER_PACK 12
#define HEADER (uint8_t)0x54
typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
}LiDARFrameTypeDef;

void cartesianToPolar(float x, float y, float &r, float &q ) {
    r = sqrt(x * x + y * y); q = atan(y / x);
}

void polarToCartesian(float r, float q, float &x, float &y) {
    x = r * cos(q); y = r * sin(q);
}
int taskRead(LiDARFrameTypeDef * frame){
  //Serial.print(Serial2.available());
  //Serial.print(":");
  if (Serial2.available() >= 48) {
    uint8_t fc;
    Serial2.readBytes(&fc, 1);
    //Serial.println(fc);
    if (fc == HEADER) {
      frame->header = HEADER;
      Serial2.readBytes(&(frame->ver_len), 10+POINT_PER_PACK*3);
      //TODO: check CRC
      return 1;
    }
  }
  return 0;
}

void printLF(LiDARFrameTypeDef* lf){
  Serial.println("\nLeedar Frame info:\n");
  Serial.print("Header: ");
  Serial.println(lf->header);
  Serial.print("val_len: ");
  Serial.println(lf->ver_len);
  Serial.print("speed: ");
  Serial.println(lf->speed);
  Serial.print("start_angle: ");
  Serial.println(lf->start_angle);
  Serial.print("points: [ ");
  for(auto x : lf->point){
    Serial.print("{");
    Serial.print(x.distance);
    Serial.print(", ");
    Serial.print(x.intensity);
    Serial.print("} ");
  }  

    Serial.print(" ]\n");
    Serial.print("end_angle: ");
    Serial.println(lf->end_angle);
    Serial.print("Timnestamp: ");
    Serial.println(lf->timestamp);
    Serial.print("crc8: ");
    Serial.println(lf->crc8);

}




void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  Serial.begin(921600);
  Serial.println("potato");
  Serial.println("potato");
  Serial2.begin(230400);
  spi_init();
  //MCP2515 mcp2515(&spi);
  MCP2515 mcp2515(&spi);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  sleep(10000);
  
  struct can_frame frame;
  frame.can_id = 0x000;
  frame.can_dlc = 4;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;

  /* send out the message to the bus and
  tell other devices this is a standard frame from 0x00. */
  Serial.println("aadafa");
  mcp2515.sendMessage(&frame);


  
  }



LiDARFrameTypeDef *lf = new LiDARFrameTypeDef;
/*
void loop() 
{
  *lf = {0};
  while (!taskRead(lf)) {
    //Serial.println(Serial2.available());
  }
  printLF(lf);
 // for (int i = 0; i < 5000; ++i) 
 taskRead(lf);
  //Serial.println("potato2");
  
}
*/




void loop() {
  Serial.println("ola");
  
  sleep(1000);
}
// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}