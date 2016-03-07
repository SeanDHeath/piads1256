package com.heath.rpi
import java.util.concurrent.TimeUnit

import ADS1256.Input.Input
import ADS1256.Register.Register
import com.pi4j.io.gpio.{PinState, RaspiPin, GpioFactory}
import com.pi4j.io.spi.{SpiMode, SpiChannel, SpiFactory}

import scala.None
import scala.collection.mutable
import scala.collection.mutable.{ListBuffer, Queue}

object ADS1256 {
  // Register Addresses
  val NUM_REGISTERS = 11
  object Register extends Enumeration {
    type Register = Value
    val STATUS    = Value(0x00)
    val MUX       = Value(0x01)
    val ADCON     = Value(0x02)
    val DRATE     = Value(0x03)
    val IO        = Value(0x04)
    val OFC0      = Value(0x05)
    val OFC1      = Value(0x06)
    val OFC2      = Value(0x07)
    val FSC0      = Value(0x08)
    val FSC1      = Value(0x09)
    val FSC2      = Value(0x0A)
  }

  // Analog Inputs
  object Input extends Enumeration {
    type Input    = Value
    val AIN0      = Value(0x00)
    val AIN1      = Value(0x01)
    val AIN2      = Value(0x02)
    val AIN3      = Value(0x03)
    val AIN4      = Value(0x04)
    val AIN5      = Value(0x05)
    val AIN6      = Value(0x06)
    val AIN7      = Value(0x07)
    val AINCOM    = Value(0x08)
  }

  // Commands
  object Command extends Enumeration {
    type Command = Value
    val WAKEUP    = Value(0x00)
    val RDATA     = Value(0x01)
    val RDATAC    = Value(0x03)
    val SDATAC    = Value(0x0F)
    val RREG      = Value(0x10)
    val WREG      = Value(0x50)
    val SELFCAL   = Value(0xF0)
    val SELFOCAL  = Value(0xF1)
    val SELFGCAL  = Value(0xF2)
    val SYSOCAL   = Value(0xF3)
    val SYSGCAL   = Value(0xF4)
    val SYNC      = Value(0xFC)
    val STANDBY   = Value(0xFD)
    val RESET     = Value(0xFE)
  }

  // DRATE Speeds
  object DataRate extends Enumeration {
    type DataRate = Value
    val DRATE_30000 = Value(0xF0) // 30,000SPS (default)
    val DRATE_15000 = Value(0xE0) // 15,000SPS
    val DRATE_7500 = Value(0xD0) // 7,500SPS
    val DRATE_3750 = Value(0xC0) // 3,750SPS
    val DRATE_2000 = Value(0xB0) // 2,000SPS
    val DRATE_1000 = Value(0xA1) // 1,000SPS
    val DRATE_500 = Value(0x92) // 500SPS
    val DRATE_100 = Value(0x82) // 100SPS
    val DRATE_60 = Value(0x72) // 60SPS
    val DRATE_50 = Value(0x63) // 50SPS
    val DRATE_30 = Value(0x53) // 30SPS
    val DRATE_25 = Value(0x43) // 25SPS
    val DRATE_15 = Value(0x33) // 15SPS
    val DRATE_10 = Value(0x20) // 10SPS
    val DRATE_5 = Value(0x13) // 5SPS
    val DRATE_2_5 = Value(0x03)// 2.5SPS
  }
  // Gain levels
  object GainLevel extends Enumeration {
    val AD_GAIN_1 = Value(0x00)// default
    val AD_GAIN_2 = Value(0x01)
    val AD_GAIN_4 = Value(0x02)
    val AD_GAIN_8 = Value(0x03)
    val AD_GAIN_16 = Value(0x04)
    val AD_GAIN_32 = Value(0x05)
    val AD_GAIN_64 = Value(0x06)
  }

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

  // These must be used to tell the ADS1256 we're talking to it - if we don't use them then we will get no information
  // from the chip
  def chip_select() = CS_PIN.low()
  def chip_release() = CS_PIN.high()

  // This stores the current register the ADS1256 is reading
  var current_positive_input = Input.AIN0
  var current_negative_input = Input.AIN1

  // Initial Config of Pi
  RESET_PIN.high()
  PDWN_PIN.high()
  chip_release()

  // Methods
  def DataReady(): Boolean = {
    val currentTime = System.currentTimeMillis()

    // Wait for drdy to go low or for timeout to occur
    while (((currentTime - System.currentTimeMillis()) < drdy_timeout) &&
      (DRDY_PIN.getState != PinState.LOW)) {
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
  def WriteRegister(start_register: Register, data: Byte): Unit = {
    // Validate the register and data length
    if (Register.values.exists(_ == start_register)) {
      var commands = Queue[Int]()
      commands += (Command.WREG.id | start_register.id)
      commands += 0x00
      commands += data

      chip_select()
      WriteSPI(commands)
      chip_release()
    } else {
      println("Write Register: Provided register address is not valid")
    }
  }

  /** Writes a string of data to the SPI bus, all Ints are truncated to Bytes
    *
    * @param values
    */
  def WriteSPI(values: Queue[Int]) = {
    println("WriteSPI: " + values)
    for (value <- values){
      spidev.write(value.toByte)
      // Need to wait between each byte
      TimeUnit.MICROSECONDS.sleep(10)
    }
  }
  def ReadSPI(n: Int): List[Byte] = {
    val arr = Array.fill[Byte](n)(0)
    spidev.write(arr, 0, arr.length).toList
  }

  /** Writes data to multiple consecutive registers on the ADS1256.
    *
    * @param start_register: Value from the Register map
    * @param data
    */
  def WriteRegisters(start_register: Register, data: List[Int]) = {
      var commands = Queue[Int]()
      // Generate the command for the appropriate register
      commands += (Command.WREG.id | start_register.id)
      // Add the number of registers to write
      commands += data.length
      // Add the data to write
      for (item <- data){
        commands += item
      }

      chip_select()
      WriteSPI(commands)
      chip_release()
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
  def SelectInput(positive_input: Input, negative_input: Input = Input.AINCOM) = {
      // Wait for DRDY before sending MUX signal
      if (DataReady()){
        var commands = Queue[Int]()
        commands += ((positive_input.id << 4) | (negative_input.id & 0x0F))
        commands += Command.SYNC.id
        commands += Command.WAKEUP.id

        chip_select()
        WriteSPI(commands)
        chip_release()
      }
  }

  /** Returns an Int containing the 24 bit analog voltage value provided by the ADS1256
    *
    */
  def ReadData(): Option[Int] = {
    // If data is ready
    if (DataReady()) {
      // Send the command
      var commands = Queue[Int]()
      commands += Command.RDATA.id
      chip_select()
      WriteSPI(commands)
      DataDelay()
      val bytes = ReadSPI(3)
      chip_release()
      println(bytes)
      val retval = (bytes(0) << 16) | (bytes(1) << 8) | (bytes(2)) & 0x0FFF
      Some(retval)
    } else {
      println("ReadData: Failed to read data")
      None
    }
  }

  def DataDelay() = {
    TimeUnit.MICROSECONDS.sleep(100)
  }


  // TODO: ReadDifferentialInput
  def ReadInput(positive_input: Input, negative_input: Input = Input.AINCOM) = {
    // We only want to switch the input if we have to
    if ((current_positive_input != positive_input) || (current_negative_input != negative_input)){
        current_positive_input = positive_input
        current_negative_input = negative_input
        SelectInput(positive_input, negative_input)
    }
    ReadData()
  }

  def ConvertVoltage(voltage: Int, vref: Double = 5.0): Double = {
    val resolution: Double = vref / 0x0FFF.toDouble
    voltage * resolution
  }

  def InitializeADS1256() = {
    var commands = Queue[Int]()
    commands += Command.WREG.id | Register.STATUS.id
    commands += 0x00
    commands += 0x04 // Auto calibration enabled
    chip_select()
    WriteSPI(commands)
    chip_release()

    commands.clear()
    commands += Command.WREG.id | Register.DRATE.id
    commands += 0x00
    commands += DataRate.DRATE_1000.id // 1k samples / sec
    chip_select()
    WriteSPI(commands)
    chip_release()
  }

  def main(args: Array[String]): Unit = {
    InitializeADS1256()

    for (input <- Input.values){
      ReadInput(input) match {
        case Some(i) => println("Input " + input.id.toString + ": " + ConvertVoltage(i).toString + "V")
        case None => println("Failed to read input " + input.id.toString)
      }
    }
  }
}
