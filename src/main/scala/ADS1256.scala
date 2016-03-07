package com.heath.rpi
import java.util.concurrent.TimeUnit

import ADS1256.Input.Input
import ADS1256.Register.Register
import akka.io.Tcp.WriteCommand
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
    val STATUS    = Value(0x0000)
    val MUX       = Value(0x0001)
    val ADCON     = Value(0x0002)
    val DRATE     = Value(0x0003)
    val IO        = Value(0x0004)
    val OFC0      = Value(0x0005)
    val OFC1      = Value(0x0006)
    val OFC2      = Value(0x0007)
    val FSC0      = Value(0x0008)
    val FSC1      = Value(0x0009)
    val FSC2      = Value(0x000A)
  }

  // Analog Inputs
  object Input extends Enumeration {
    type Input    = Value
    val AIN0      = Value(0x0000)
    val AIN1      = Value(0x0001)
    val AIN2      = Value(0x0002)
    val AIN3      = Value(0x0003)
    val AIN4      = Value(0x0004)
    val AIN5      = Value(0x0005)
    val AIN6      = Value(0x0006)
    val AIN7      = Value(0x0007)
    val AINCOM    = Value(0x0008)
  }

  // Commands
  object Command extends Enumeration {
    type Command = Value
    val WAKEUP    = Value(0x0000)
    val RDATA     = Value(0x0001)
    val RDATAC    = Value(0x0003)
    val SDATAC    = Value(0x000F)
    val RREG      = Value(0x0010)
    val WREG      = Value(0x0050)
    val SELFCAL   = Value(0x00F0)
    val SELFOCAL  = Value(0x00F1)
    val SELFGCAL  = Value(0x00F2)
    val SYSOCAL   = Value(0x00F3)
    val SYSGCAL   = Value(0x00F4)
    val SYNC      = Value(0x00FC)
    val STANDBY   = Value(0x00FD)
    val RESET     = Value(0x00FE)
  }

  // DRATE Speeds
  object DataRate extends Enumeration {
    type DataRate = Value
    val DRATE_30000 = Value(0x00F0) // 30,000SPS (default)
    val DRATE_15000 = Value(0x00E0) // 15,000SPS
    val DRATE_7500 = Value(0x00D0) // 7,500SPS
    val DRATE_3750 = Value(0x00C0) // 3,750SPS
    val DRATE_2000 = Value(0x00B0) // 2,000SPS
    val DRATE_1000 = Value(0x00A1) // 1,000SPS
    val DRATE_500 = Value(0x0092) // 500SPS
    val DRATE_100 = Value(0x0082) // 100SPS
    val DRATE_60 = Value(0x0072) // 60SPS
    val DRATE_50 = Value(0x0063) // 50SPS
    val DRATE_30 = Value(0x0053) // 30SPS
    val DRATE_25 = Value(0x0043) // 25SPS
    val DRATE_15 = Value(0x0033) // 15SPS
    val DRATE_10 = Value(0x0020) // 10SPS
    val DRATE_5 = Value(0x0013) // 5SPS
    val DRATE_2_5 = Value(0x0003)// 2.5SPS
  }

  object Status extends Enumeration {
    type Status = Value
    val ORDER_LSB = Value(1 << 3)
    val ACAL_ON = Value(1 << 2)
    val BUFFER_ON = Value(1 << 1)
  }

  object ADControl extends Enumeration {
    type ADControl = Value
    val CLK_FCLKIN = Value(1 << 5)
    val CLK_HALF_FCLKIN = Value(2 << 5)
    val CLK_FOURTH_FCLKIN = Value(3 << 5)
    val SD_500nA = Value(1 << 3)
    val SD_2uA = Value(2 << 3)
    val SD_10uA = Value(3 << 3)
    val GAIN_2 = Value(0x0001)
    val GAIN_4 = Value(0x0002)
    val GAIN_8 = Value(0x0003)
    val GAIN_16 = Value(0x0004)
    val GAIN_32 = Value(0x0005)
    val GAIN_64 = Value(0x0006)
  }

  // Defaults for RPi 2
  var spi_frequency = 1000000
  // 1MHz
  var spi_channel = SpiChannel.getByNumber(0)
  var spi_mode = SpiMode.MODE_1
  var drdy_timeout = 500 // ms
  var adc_gain = 1 // to adjust this, write to the

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
  def chip_select() = {
    CS_PIN.low()
    TimeUnit.MICROSECONDS.sleep(5)
  }
  def chip_release() = {
    TimeUnit.MICROSECONDS.sleep(1)
    CS_PIN.high()
  }

  // This stores the current register the ADS1256 is reading
  var current_positive_input = Input.AIN0
  var current_negative_input = Input.AIN1

  // Initial Config of Pi
  RESET_PIN.high()
  PDWN_PIN.high()
  chip_release()

  // Methods
  def DataReady(): Boolean = {
    val startTime = System.currentTimeMillis()
    var currentTime = System.currentTimeMillis()

    println("DataReady: " + currentTime + ", " + startTime)
    println(DRDY_PIN.getState)

    // Wait for drdy to go low or for timeout to occur
    while (((currentTime - startTime) < drdy_timeout) &&
      (DRDY_PIN.getState != PinState.LOW)) {
      currentTime = System.currentTimeMillis()
      println("waiting for data...")
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
      WriteSPI(Command.WREG.id | start_register.id)
      WriteSPI(0x00)
      WriteSPI(data)
      chip_release()
      TimeUnit.MICROSECONDS.sleep(1)
    } else {
      println("Write Register: Provided register address is not valid")
    }
  }

  /** Writes a string of data to the SPI bus, all Ints are truncated to Bytes
    *
    * @param value
    */
  def WriteSPI(value: Int) = {
    println("WriteSPI: " + value)
    spidev.write(value.toByte)
  }
  def ReadSPI(n: Int) = {
    val arr = Array.fill[Byte](n)(0xFF.toByte)
    val result = spidev.write(arr, 0, arr.length).toList
    result.map{i: Byte => i.toInt & 0x00FF}
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
        chip_select()
        WriteRegister(Register.MUX, (positive_input.id << 4) | negative_input.id)
        Sync()
        Wakeup()
        chip_release()
      }
  }

  def Wakeup() = {
    WriteSPI(Command.WAKEUP.id)
  }

  def Sync() = {
    WriteSPI(Command.SYNC.id)
    TimeUnit.MICROSECONDS.sleep(4)
  }

  def WriteRegister(register: Register, value: Int) = {
    WriteSPI(Command.WREG.id | register.id)
    WriteSPI(0x00)
    WriteSPI(value)
    TimeUnit.MICROSECONDS.sleep(1)
  }

  def ReadRegister(register: Register) = {
    WriteSPI(Command.RREG.id | register.id)
    WriteSPI(0x00)
    DataDelay()
    val value = ReadSPI(1)
    TimeUnit.MICROSECONDS.sleep(1)
    value
  }

  def DataDelay() = {
    TimeUnit.MICROSECONDS.sleep(10)
  }

  def SelfCal() = {
    WriteSPI(Command.SELFCAL.id)
    DataReady()
  }

  /** Returns an Int containing the 24 bit analog voltage value provided by the ADS1256
    *
    */
  def ReadData(): Option[Int] = {
    // If data is ready
    if (DataReady()) {
      // Send the command
      chip_select()
      WriteSPI(Command.RDATA.id)
      TimeUnit.MICROSECONDS.sleep(10)
      val bytes = ReadSPI(3)
      chip_release()

      println(bytes)
      val msb = bytes(0)<< 24
      val midb = bytes(1)<< 16
      val lsb = bytes(2)<< 8
      val retval = (msb | midb | lsb) >> 8
      println(retval)
      Some(retval)
    } else {
      println("ReadData: Failed to read data")
      None
    }
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

  def ConvertVoltage(voltage: Int, vref: Double = 5): Double = {
    val resolution = (vref * 2) / 0x00FFFFFF.toDouble
    voltage * resolution
  }

  def InitializeADS1256() = {
    var commands = Queue[Int]()
    chip_select()
    WriteRegister(Register.STATUS, Status.ACAL_ON.id)
    WriteRegister(Register.DRATE, DataRate.DRATE_1000.id)
    println("Read Register: " + ReadRegister(Register.STATUS))
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
