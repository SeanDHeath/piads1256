import akka.actor.Actor
import com.pi4j.io.gpio.{PinState, RaspiPin, GpioFactory}
import com.pi4j.io.spi.{SpiMode, SpiChannel, SpiFactory}

import scala.collection.mutable.ListBuffer

object ADS1256 {
  // Register Addresses
  val NUM_REGISTERS = 11
  val Register = Map[String, Byte](
    "STATUS" -> 0x00.toByte,
    "MUX" -> 0x01.toByte,
    "ADCON" -> 0x02.toByte,
    "DRATE" -> 0x03.toByte,
    "IO" -> 0x04.toByte,
    "OFC0" -> 0x05.toByte,
    "OFC1" -> 0x06.toByte,
    "OFC2" -> 0x07.toByte,
    "FSC0" -> 0x08.toByte,
    "FSC1" -> 0x09.toByte,
    "FSC2" -> 0x0A.toByte
  )

  // Analog Inputs
  val Input = Map[String, Byte](
    "AIN0" -> 0x00.toByte,
    "AIN1" -> 0x01.toByte,
    "AIN2" -> 0x02.toByte,
    "AIN3" -> 0x03.toByte,
    "AIN4" -> 0x04.toByte,
    "AIN5" -> 0x05.toByte,
    "AIN6" -> 0x06.toByte,
    "AIN7" -> 0x07.toByte,
    "AINCOM" -> 0x80.toByte
  )

  // Commands
  val Command = Map[String, Byte](
    "WAKEUP" -> 0x00.toByte, // Completes SYNC and exits standby mode
    "RDATA" -> 0x01.toByte, // Read data
    "RDATAC" -> 0x03.toByte, // Start read data continuously
    "SDATAC" -> 0x0F.toByte, // Stop read data continuously
    "RREG" -> 0x10.toByte, // Read from register
    "WREG" -> 0x50.toByte, // Write to register
    "SELFCAL" -> 0xF0.toByte, // Offset and gain self-calibration
    "SELFOCAL" -> 0xF1.toByte, // Offset self-calibration
    "SELFGCAL" -> 0xF2.toByte, // Gain self-calibration
    "SYSOCAL" -> 0xF3.toByte, // System offset calibration
    "SYSGCAL" -> 0xF4.toByte, // System gain calibration
    "SYNC" -> 0xFC.toByte, // Synchronize the A/D conversion
    "STANDBY" -> 0xFD.toByte, // Begin standby mode
    "RESET" -> 0xFE.toByte // Reset to power-on values
  )

  // DRATE Speeds
  val DataRate = Map[String, Byte](
    "DRATE_30000" -> 0xF0.toByte, // 30,000SPS (default)
    "DRATE_15000" -> 0xE0.toByte, // 15,000SPS
    "DRATE_7500" -> 0xD0.toByte, // 7,500SPS
    "DRATE_3750" -> 0xC0.toByte, // 3,750SPS
    "DRATE_2000" -> 0xB0.toByte, // 2,000SPS
    "DRATE_1000" -> 0xA1.toByte, // 1,000SPS
    "DRATE_500" -> 0x92.toByte, // 500SPS
    "DRATE_100" -> 0x82.toByte, // 100SPS
    "DRATE_60" -> 0x72.toByte, // 60SPS
    "DRATE_50" -> 0x63.toByte, // 50SPS
    "DRATE_30" -> 0x53.toByte, // 30SPS
    "DRATE_25" -> 0x43.toByte, // 25SPS
    "DRATE_15" -> 0x33.toByte, // 15SPS
    "DRATE_10" -> 0x20.toByte, // 10SPS
    "DRATE_5" -> 0x13.toByte, // 5SPS
    "DRATE_2_5" -> 0x03.toByte // 2.5SPS
  )
  // Gain levels
  val GainLevel = Map[String, Byte](
    "AD_GAIN_1" -> 0x00.toByte, // default
    "AD_GAIN_2" -> 0x01.toByte,
    "AD_GAIN_4" -> 0x02.toByte,
    "AD_GAIN_8" -> 0x03.toByte,
    "AD_GAIN_16" -> 0x04.toByte,
    "AD_GAIN_32" -> 0x05.toByte,
    "AD_GAIN_64" -> 0x06.toByte
  )

  // Defaults for RPi 2
  var spi_frequency = 1000000
  // 1MHz
  var spi_channel = SpiChannel.getByNumber(0)
  var spi_mode = SpiMode.MODE_1
  var drdy_timeout = 500 // ms

  // Open the new device
  val spidev = SpiFactory.getInstance(spi_channel, spi_frequency, spi_mode)

  // Set up pins
  val gpio = GpioFactory.getInstance()
  val DRDY_PIN = gpio.provisionDigitalInputPin(RaspiPin.GPIO_00)
  val RESET_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_01)
  val PDWN_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02)
  val CS_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_03)

  def chip_select() = CS_PIN.low()

  def chip_release() = CS_PIN.high()

  val null_byte = 0x00.toByte

  // Initial Config of Pi
  RESET_PIN.high()
  PDWN_PIN.high()
  chip_release()

  // Methods
  def DataReady(): Boolean = {
    val currentTime = System.currentTimeMillis()

    // Wait for drdy to go low or for timeout to occur
    while (((currentTime - System.currentTimeMillis()) < drdy_timeout) &&
      (DRDY_PIN.getState() != PinState.LOW)) {
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

  /** Writes data to an ADS1256 register through the SPI bus.
    *
    * @param start_register
    * @param data
    */
  def WriteRegister(start_register: Byte, data: Byte) = {
    // Validate the register and data length
    if (Register.values.exists(_ == start_register)) {
      chip_select()
      // Generate the command for the appropriate register
      val cmd: Byte = (Command("WREG") | start_register).toByte
      // Write the start register and command
      spidev.write(cmd)
      // Write the number of follow on registers to write (determined by the length of data)
      spidev.write(0x00.toByte)
      // Write the register contents
      spidev.write(data)
      chip_release()
    } else {
      println("Write Register: Provided register address is not valid")
    }
  }

  /** Writes data to multiple consecutive registers on the ADS1256.
    *
    * @param start_register
    * @param data
    */
  def WriteRegisters(start_register: Byte, data: List[Byte]) = {
    // Validate the register and data length
    if (Register.values.exists(_ == start_register)) {
      // Ensure the length of the data is shorter than the number of registers provided by the ADS1256
      if (data.length <= NUM_REGISTERS) {
        chip_select()

        // Generate the command for the appropriate register
        val cmd: Byte = (Command("WREG") | start_register).toByte

        // Write the start register and command
        spidev.write(cmd)
        // Write the number of follow on registers to write (determined by the length of data)
        spidev.write(data.length.toByte)
        // Write the register contents
        for (byte <- data) {
          spidev.write(byte)
        }
        chip_release()
      } else {
        println("Write Register: Invalid data length")
      }
    } else {
      println("Write Register: Provided register address is not valid")
    }
  }

  // The chip default is a differential reading between AIN0 and AIN1, this defaults to reading a single ended read
  // on positive_input
  /** Selects which analog input to read from the ADS1256
    *
    * This function writes to the MUX register and tells the ADS1256 which input to read from. If only a positive input
    * is provided then it will provide the voltage on that pin with reference to AINCOM. If two inputs are provided
    * then it will provide the differential voltage between them.
    *
    * @param positive_input
    * @param negative_input
    */
  def SelectInput(positive_input: Byte, negative_input: Byte = Input("AINCOM")) = {
    // The top 4 bits indicate the positive input, the bottom 4 bits indicate the negative input, default to AINCOM (GND)
    if (Input.values.exists(_ == positive_input) && Input.values.exists(_ == negative_input)) {
      val data = ((positive_input << 4) | (negative_input & 0x0F)).toByte
      WriteRegister(Register("MUX"), data)
    } else {
      println("Select Input: Invalid inputs selected")
    }
  }

  /** Returns an Int containing the 24 bit analog voltage value provided by the ADS1256
    *
    */
  def ReadData(): Int = {
    // If data is ready
    if (DataReady()) {
      chip_select()
      // Send the command
      val read_bytes = Array[Byte](0, 0, 0)
      spidev.write(Command("RDATA"))
      // Read back 24 bytes of data
      val bytes = spidev.write(read_bytes, 0, 3)
      println(((bytes(0) << 16) | (bytes(1) << 8) | bytes(2)) & (0x0FFF))
      ((bytes(0) << 16) | (bytes(1) << 8) | bytes(2)) & (0x0FFF)
    } else {
      println("Could not read ADC")
      0
    }
  }

  // TODO: ReadDifferentialInput
  def ReadSingleInput(input: Byte) = {
    if (Input.values.exists(_ == input)){
      SelectInput(input)
      ReadData()
    } else {
      println("ReadInput: Invalid input number")
      0
    }
  }

  def ReadSingleInputs(inputs: List[Byte]) = {
    // Ensure the inputs provided are valid input numbers
    val values = new ListBuffer[Int]()
    for (input <- inputs) {
      values += ReadSingleInput(input)
    }
    values.toList
  }

  def ConvertVoltage(voltage: Int, vref: Double = 5.0): Double = {
    val resolution: Double = vref / 0x0FFF
    voltage * resolution
  }

  def main(args: Array[String]): Unit = {

    /*
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
    */

    val results = ReadSingleInputs(Input.values.toList)
    val converted = results.map{voltage: Int => ConvertVoltage(voltage)}
    converted.map{value: Double => println(value)}
    results.map{result: Int => println(result)}
  }
}
