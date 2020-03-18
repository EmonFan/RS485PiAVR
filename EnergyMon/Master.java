
import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.RaspiPin;

import com.pi4j.io.serial.Serial;
import com.pi4j.io.serial.SerialDataEvent;
import com.pi4j.io.serial.SerialDataListener;
import com.pi4j.io.serial.SerialFactory;
import com.pi4j.io.serial.SerialPortException;

public class Master {

  private static final int PKT_LEADIN_BYTE = 0x99;
  private static final int PKT_START_BYTE  = 0xaa;
  private static final int SLAVE_BASE      = 0xa0;
  private static final int MASTER_ID       = 0x0b;
  private static final int PKTF_ACK        = 0x01;
  private static final int PKTF_NACK       = 0x02;
  private static final int PKTF_REQACK     = 0x04;
  private static final int PKTF_REQID      = 0x08;
  private static final int PKTF_COMMAND    = 0x10;
  private static final int PKTF_RESERVED   = 0x20;

  private static final int PKT_N_LEADIN    = 4;   //Bytes before PKT_FLAG

  private static final int PKT_LEADIN      = 0x00;
  private static final int PKT_START       = 0x01;
  private static final int PKT_DEST        = 0x02;
  private static final int PKT_SOURCE      = 0x03;
  private static final int PKT_FLAG        = 0x04;
  private static final int PKT_LENGTH      = 0x05;
  private static final int PKT_DATA        = 0x06;
  private static final int PKT_CHECKSUM    = 0x07;
  private static final int PKT_VALID       = 0x08;
  private static final int PKT_NOT_VALID   = 0x09;

  //Define the supported slave commands
  private static final int ENUMERATE_COUNT = 0xD0;
  private static final int ENUMERATE_ITEM  = 0xD1;
  private static final int TRANSMIT_ITEM   = 0xD2;

  public static void main(String[] args) throws InterruptedException {

  System.out.println("Master ... started.");

  // create gpio controller
  final GpioController gpio = GpioFactory.getInstance();

  // provision gpio pin #07 as an output pin for our RS485 transmit mode pin
  final GpioPinDigitalOutput readEnable = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_07);

  // create an instance of the serial communications class
  final Serial serial = SerialFactory.createInstance();

 // create and register the serial data listener
  serial.addListener(new SerialDataListener() {
      @Override
      public void dataReceived(SerialDataEvent event) {
        // print out the data received to the console
        String sData = event.getData();
        char [] charArray = sData.toCharArray();
          
        StringBuilder sb = new StringBuilder();
        for (char b : charArray) {
            sb.append(String.format("%02X:", (byte)b));
        }

        System.out.println(sb.toString());
      }
  });


  //Enable RS485 transmit mode
  readEnable.high();

  try {
    // open the default serial port provided on the GPIO header
    serial.open(Serial.DEFAULT_COM_PORT, 230400);

    try {
      // write individual bytes to the serial transmit buffer
      //[PKT_LEADIN_BYTE, PKT_START_BYTE, slaveID, MASTER_ID, PKTF_REQID, 0x01, 0x00])
      serial.write((byte) PKT_LEADIN_BYTE);
      serial.write((byte) PKT_START_BYTE);
      serial.write((byte) 0xa5);
      serial.write((byte) 0x0b);
      serial.write((byte) PKTF_REQID);
      serial.write((byte) 0x01);
      serial.write((byte) 0x00);
      serial.write((byte) 0xb8);
    }
    catch(IllegalStateException ex){
        ex.printStackTrace();                    
    }
  }
  catch(SerialPortException ex) {
    System.out.println(" ==>> SERIAL SETUP FAILED : " + ex.getMessage());
    return;
  }

  //Enable RS485 receive mode
  readEnable.low();

  System.out.println("Sleeping for 3 seconds");
  // wait 3 second before continuing
  Thread.sleep(3000);

  }
}
