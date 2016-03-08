package com.heath.rpi
import java.util.concurrent.TimeUnit

import ADS1256.Input.Input
import ADS1256.Register.Register
import com.pi4j.io.gpio.{PinState, RaspiPin, GpioFactory}
import com.pi4j.io.spi.{SpiMode, SpiChannel, SpiFactory}

import scala.collection.mutable.ListBuffer

object ADS1256 {

  /*
  TODO: Implement GPIO
  TODO: SetDataRate()
  TODO: SetGain()
  TODO: SetAutoCalibration()
  TODO: ReadID()
  TODO: SetBitOrder()
  TODO: SetBufferEnable()
  TODO: SetCLKOut()
  TODO: SetSensorDetectCurrentSources()
   */

  val debugging = false

  /** Used to print debug messages
    *
    * @param s: String to print
    */
  def dprintln(s: String) = {
    if (debugging) {
      println("DEBUG: " + s)
    }
  }

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

  // Open the new SPI device
  val spidev = SpiFactory.getInstance(spi_channel, spi_frequency, spi_mode)

  // Set up pins
  val gpio      = GpioFactory.getInstance()
  val DRDY_PIN  = gpio.provisionDigitalInputPin(RaspiPin.GPIO_00)
  val RESET_PIN = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_01)
  val PDWN_PIN  = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02)
  val CS_PIN    = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_03)

  // These must be used to tell the ADS1256 we're talking to it - if we don't use them then we will get no information
  // from the chip
  def chip_select() = {
    CS_PIN.low()
    TimeUnit.MICROSECONDS.sleep(1)
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

  /** Returns a boolean value indicating if the DRDY pin is indicating that there is data ready (true) or if there is
    * an error (false)
    */
  def DataReady(): Boolean = {
    dprintln("DataReady")
    val startTime = System.currentTimeMillis()
    var currentTime = System.currentTimeMillis()

    dprintln("DRDY - " + DRDY_PIN.getState)

    while (((currentTime - startTime) < drdy_timeout) &&
      (DRDY_PIN.getState != PinState.LOW)) {
      currentTime = System.currentTimeMillis()
      dprintln("waiting for data...")
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

  /** Writes a string of data to the SPI bus, all Ints are truncated to Bytes
    *
    * @param value: Integer value, only the bottom 8 bits are kept
    */
  def WriteSPI(value: Int) = {
    spidev.write(value.toByte)
  }

  /** Returns a List[Int] with the lower 8 bits of each Int containing the value read from the SPI bus
    *
    * @param n: number of bytes to read from the SPI bus
    */
  def ReadSPI(n: Int) = {
    val arr = Array.fill[Byte](n)(0xFF.toByte)
    val result = spidev.write(arr, 0, arr.length).toList
    result.map{i: Byte => i.toInt & 0x00FF}
  }

  /** Selects which analog input to read from the ADS1256
    *
    * This function writes to the MUX register and tells the ADS1256 which input to read from. If only a positive input
    * is provided then it will provide the voltage on that pin with reference to AINCOM. If two inputs are provided
    * then it will provide the differential voltage between them.
    *
    * @param positive_input: The analog input to be used as the positive input
    * @param negative_input: The analog input to be used as the negative input, AINCOM by default
    */
  def SelectInput(positive_input: Input, negative_input: Input = Input.AINCOM) = {
      // Wait for DRDY before sending MUX signal
      if (DataReady()){
        chip_select()
        // Executes the sequence to change registers outlined on pg 21 of the ADS1256 datasheet
        WriteRegister(Register.MUX, (positive_input.id << 4) | negative_input.id)
        Sync()
        Wakeup()
        chip_release()
      }
  }

  /** Sends the WAKEUP command to the ADS1256
    */
  def Wakeup() = {
    WriteSPI(Command.WAKEUP.id)
  }

  /** Sends the SYNC command to the ADS1256 and delays 4us as required by the chip. This information is outlined on
    * pg 6 of the datasheet.
    */
  def Sync() = {
    WriteSPI(Command.SYNC.id)
    TimeUnit.MICROSECONDS.sleep(4)
  }

  /** Writes the bottom 8 bits of the provided value to the register provided. Delays the appropriate amount of time
    * as outlined on pg6 of the datasheet for the ADS1256.
    *
    * @param register: Register to write to
    * @param value: Value to write to that register
    */
  def WriteRegister(register: Register, value: Int) = {
    WriteSPI(Command.WREG.id | register.id)
    WriteSPI(0x00)
    WriteSPI(value)
    TimeUnit.MICROSECONDS.sleep(1)
  }

  /** Returns an Int with the bottom 8 bytes indicating the value of the register.
    *
    * @param register: The register to read
    * @return
    */
  def ReadRegister(register: Register) = {
    WriteSPI(Command.RREG.id | register.id)
    WriteSPI(0x00)
    DataDelay()
    val value = ReadSPI(1).head
    TimeUnit.MICROSECONDS.sleep(1)
    value.toInt & 0x00FF
  }

  /** Delay to execute after writing a command that requests data. This value is calculated from pg6 of the ADS1256
    * datasheet.
    */
  def DataDelay() = {
    TimeUnit.MICROSECONDS.sleep(10)
  }

  /** Writes the SELFCAL command to the ADS1256 and waits for the DRDY line to go high per instructions on pg6 of the
    * ADS1256 datasheet.
    */
  def SelfCal() = {
    WriteSPI(Command.SELFCAL.id)
    DataReady()
  }

  /** Returns a signed Int containing the 24 bit analog voltage value provided by the ADS1256
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

      // First we shift over to the end of the Int size (32 bits) to preserve the sign of the measurement.
      val msb = bytes.head << 24
      val midb = bytes(1)<< 16
      val lsb = bytes(2)<< 8
      // Then all values are ORed together and shifted back to the right (which preserves the sign)
      val retval = (msb | midb | lsb) >> 8
      Some(retval)
    } else {
      println("Failed to read data")
      None
    }
  }

  /** Returns an Option[Int] containing the value read from the currently selected ADC input.
    *
    * @param positive_input: The input to use as the positive measurement
    * @param negative_input: The input to use as the negative measurement
    */
  def ReadInput(positive_input: Input, negative_input: Input = Input.AINCOM) = {
    // We only want to switch the input if we have to
    if ((current_positive_input != positive_input) || (current_negative_input != negative_input)){
        current_positive_input = positive_input
        current_negative_input = negative_input
        SelectInput(positive_input, negative_input)
    }
    ReadData()
  }

  /** Converts the 24 bit number to a voltage using the conversion indicated in Table 16 on pg23 of the ADS1256
    * data sheet.
    *
    * @param voltage: 24 bit signed integer containing the measured value from the ADC
    * @param vref: The current reference voltage, defaults to 5V
    * @return
    */
  def ConvertVoltage(voltage: Int, vref: Double = 5): Double = {
    val resolution = (vref * 2) / 0x00FFFFFF.toDouble
    voltage * resolution
  }

  /** Initializes the ADS1256 with the following parameters enabled:
    * Autocalibration on - page 26 of the ADS1256 datasheet
    * Data rate - 1k samples per second
    */
  def InitializeADS1256() = {
    chip_select()
    WriteRegister(Register.STATUS, Status.ACAL_ON.id)
    WriteRegister(Register.DRATE, DataRate.DRATE_1000.id)
    chip_release()
  }
}
