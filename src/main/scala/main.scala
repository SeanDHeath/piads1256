import akka.actor.Actor
import com.pi4j.io.gpio.{PinState, RaspiPin, GpioFactory}
import com.pi4j.io.spi.{SpiMode, SpiChannel, SpiFactory}

object ADS1256 {

  // Register Addresses
  val REG_STATUS  = 0x00
  val REG_MUX     = 0x01
  val REG_ADCON   = 0x02
  val REG_DRATE   = 0x03
  val REG_IO      = 0x04
  val REG_OFC0    = 0x05
  val REG_OFC1    = 0x06
  val REG_OFC2    = 0x07
  val REG_FSC0    = 0x08
  val REG_FSC1    = 0x09
  val REG_FSC2    = 0x0A
  val NUM_REG     = 11

  // Analog Inputs
  val AIN0    = 0x00
  val AIN1    = 0x01
  val AIN2    = 0x02
  val AIN3    = 0x03
  val AIN4    = 0x04
  val AIN5    = 0x05
  val AIN6    = 0x06
  val AIN7    = 0x07
  val AINCOM  = 0x80

  // Commands
  val CMD_WAKEUP  = 0x00 // Completes SYNC and exits standby mode
  val CMD_RDATA   = 0x01 // Read data
  val CMD_RDATAC  = 0x03 // Start read data continuously
  val CMD_SDATAC  = 0x0F // Stop read data continuously
  val CMD_RREG    = 0x10 // Read from register
  val CMD_WREG    = 0x50 // Write to register
  val CMD_SELFCAL = 0xF0 // Offset and gain self-calibration
  val CMD_SELFOCAL= 0xF1 // Offset self-calibration
  val CMD_SELFGCAL= 0xF2 // Gain self-calibration
  val CMD_SYSOCAL = 0xF3 // System offset calibration
  val CMD_SYSGCAL = 0xF4 // System gain calibration
  val CMD_SYNC    = 0xFC // Synchronize the A/D conversion
  val CMD_STANDBY = 0xFD // Begin standby mode
  val CMD_RESET   = 0xFE // Reset to power-on values


  // DRATE Speeds
  val DRATE_30000 = 0xF0 // 30,000SPS (default)
  val DRATE_15000 = 0xE0 // 15,000SPS
  val DRATE_7500  = 0xD0 // 7,500SPS
  val DRATE_3750  = 0xC0 // 3,750SPS
  val DRATE_2000  = 0xB0 // 2,000SPS
  val DRATE_1000  = 0xA1 // 1,000SPS
  val DRATE_500   = 0x92 // 500SPS
  val DRATE_100   = 0x82 // 100SPS
  val DRATE_60    = 0x72 // 60SPS
  val DRATE_50    = 0x63 // 50SPS
  val DRATE_30    = 0x53 // 30SPS
  val DRATE_25    = 0x43 // 25SPS
  val DRATE_15    = 0x33 // 15SPS
  val DRATE_10    = 0x20 // 10SPS
  val DRATE_5     = 0x13 // 5SPS
  val DRATE_2_5   = 0x03 // 2.5SPS

  // Gain levels
  val AD_GAIN_1   = 0x00 // default
  val AD_GAIN_2   = 0x01
  val AD_GAIN_4   = 0x02
  val AD_GAIN_8   = 0x03
  val AD_GAIN_16  = 0x04
  val AD_GAIN_32  = 0x05
  val AD_GAIN_64  = 0x06

  // Defaults for RPi 2
  val spi_frequency   = 1000000 // 1MHz
  val spi_channel = SpiChannel.getByNumber(0)
  val spi_mode    = SpiMode.MODE_1
  val drdy_timeout = 500 // ms

  // Open the new device
  val spidev = SpiFactory.getInstance(spi_channel, spi_frequency, spi_mode)

  // Set up pins
  val gpio = GpioFactory.getInstance()
  val DRDY_PIN = gpio.provisionDigitalInputPin(RaspiPin.GPIO_00)
  val RESET_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_01)
  val PDWN_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02)
  val CS_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_03)

  // Methods
  def DRDY: Boolean = {
    val currentTime = System.currentTimeMillis()

    // Wait for drdy to go low or for timeout to occur
    while (((currentTime - System.currentTimeMillis()) < drdy_timeout) &&
      (DRDY_PIN.getState() != PinState.LOW)){
    }

    // If we timed out
    if ((currentTime - System.currentTimeMillis()) >= drdy_timeout) {
      println("DRDY Timeout")
      false
    }
    else {
      true
    }
  }
  def ReadADC = {
    if (DRDY){

    } else {
      println("Could not read ADC")
    }

  }

  def main(args: Array[String]): Unit = {

    // Set up SPI


    RESET_PIN.high()
    PDWN_PIN.high()
    CS_PIN.low()

    val REG_STATUS = 0x00
    val CMD_RREG = 0x10
    val data = (CMD_RREG | REG_STATUS)

    val result = spi.write(data.asInstanceOf[Byte])

    println(data)
    println(result)

    var count = 0
    val byte = 0x00

    while(count < 10){
      val result: Array[Byte] = spi.write(byte.asInstanceOf[Byte])
      println(result)
      println(result(0))
      count += 1
    }
  }

  class SPIActor extends Actor {

    // Defaults

    def receive = {
      case "write" =>
    }



  }

}
