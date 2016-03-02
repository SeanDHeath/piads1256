import com.pi4j.io.gpio.{RaspiPin, GpioFactory}
import com.pi4j.io.spi.{SpiMode, SpiChannel, SpiFactory}

object ADS1256 {
  def main(args: Array[String]): Unit = {

    // Set up SPI
    val speed = 1000000 // 1MHz
    val channel = SpiChannel.getByNumber(0)
    val mode = SpiMode.MODE_1

    val spi = SpiFactory.getInstance(channel, speed, mode)

    // Set up CS
    val gpio = GpioFactory.getInstance()
    val DRDY_PIN = gpio.provisionDigitalInputPin(RaspiPin.GPIO_00)
    val RESET_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_01)
    val PDWN_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02)
    val CS_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_03)

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
      var result: Array[Byte] = spi.write(byte.asInstanceOf[Byte])
      println(result)
      println(result(0))
      count += 1
    }
  }

}
