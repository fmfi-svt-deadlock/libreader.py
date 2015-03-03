import serial
import struct
from enum import Enum
from .utils.structparse import mystruct, t, STRUCT_FORMAT

class PacketId(Enum):
    INVALID_PACKET        = 0x00
    ANSWER_TO_RESET       = 0x01
    CONTINUE_BOOT         = 0x02
    FIRMWARE_UPGRADE      = 0x03
    FW_UPGRADE_READY      = 0x04
    FW_WRITE              = 0x05
    FW_WRITE_ACK          = 0x06
    FW_UPDATE_FINISH      = 0x07
    GET_STATUS            = 0x08
    SET_LED               = 0x09
    ACK                   = 0x0A
    BEEP                  = 0x0B
    RFID_SEND             = 0x0C
    RFID_SEND_COMPLETE    = 0x0D
    RX_ERROR              = 0xFF

class ResponseLength(Enum):
    RESPONSE_ATR          = 5
    TRANSMIT_OVERHEAD     = 3

class ResetException(Exception):
        """The reader has been unexpectedly reset."""
        pass

class ReaderError(Exception):
    """The reader did something wrong and MUST be reset."""
    pass

class CorruptedPacketException(Exception):
    pass

class Reader:
    class Leds(Enum):
        RED_LED     = 0
        GREEN_LED   = 1
        BLUE_LED    = 2

    BAUDRATE   = 19200
    BYTESIZE   = serial.EIGHTBITS
    PARITY     = serial.PARITY_EVEN
    STOPBITS   = serial.STOPBITS_ONE
    RX_TIMEOUT = 0.5

    MAX_BEEP_LENGTH = 8
    MAX_RFID_PAYLOAD = 128

    PacketHead = mystruct('PacketHead',
                          ['id',     'length'],
                          [ t.uint8,  t.uint8])

    def __init__(self, port):
        self.port = serial.Serial(port, self.BAUDRATE, self.BYTESIZE,
                                  self.PARITY, self.STOPBITS, self.RX_TIMEOUT);

    def _transmit_packet(self, packet_id, payload):
        checksum =  packet_id.value
        checksum ^= len(payload)
        for byte in payload:
            checksum ^= byte
        self.port.write(bytes([packet_id.value, len(payload)]) + payload
                        + bytes([checksum]))

    def _checksum_ok(self, head, payload, checksum):
        check =  head.id
        check ^= head.length
        for byte in payload:
            check ^= byte
        return check == checksum

    def _expect_packet(self, length):
        try:
            p, payload = self.PacketHead.unpack_from(self.port.read(length))
        except struct.error as e:
            # The struct probably cannot be unpacked due to a corrupted
            # packet
            raise CorruptedPacketException() from e
        payload, checksum = payload[:-1], payload[-1]
        if not self._checksum_ok(p, payload, checksum):
            raise CorruptedPacketException()
        return p, payload

    def _transceive_with_retry(self, packet_id, payload, resp_length):
        """Transmits the packet, receives response while handling Rx errors.

        Attempts to transmit the packet, retransmitting it up to 2 times
        if RxError is received.
        After a successful transmission, it attempts to receive the response.
        If the response is corrupted, it transmits RxError up to 2 times.
        If this proces fails, this function raises ReaderError.
        """
        self._check_atr()
        for i in range(0, 2):
            try:
                self._transmit_packet(packet_id, payload)
                p, payload = self._expect_packet(resp_length)
                if p.id == PacketId.RX_ERROR:
                    # The reader does not understand, retransmit
                    pass
                else:
                    return (p, payload)
            except (CorruptedPacketException, serial.SerialTimeoutException):
                # Corrupted response to our request, request retransmission
                for j in range(0, 2):
                    self._transmit_packet(PacketId.RX_ERROR, b'')
                    try:
                        p, payload = self._expect_packet(resp_length)
                        if p.id == PacketId.RX_ERROR:
                            # RxError was a response to our RxError. This
                            # behaviour is unacceptable, let's punish the
                            # reader by resetting it!
                            raise ReaderError()
                        else:
                            return (p, payload)
                    except (CorruptedPacketException,
                            serial.SerialTimeoutException):
                        # Again, corrupted response. Retry
                        pass
        # If we got here, we've had 3 consectutive failed receive
        # attempts. Reader, how dare you! We will reset him for doing
        # this to us!
        raise ReaderError()

    def _check_atr(self):
        if self.port.inWaiting() == 5:
            # Trouble is waiting for us in the buffer; there should be
            # nothing there
            # HACK ALERT: We must release sooner than we are ready.
            # This listens for ACK instead of the ATR as the preliminary
            # version does not have a bootloader
            try:
                self._expect_packet(5)
            except serial.SerialTimeoutException:
                # Now we are in serious trouble!
                raise ReaderError()
            raise ResetException()

    def set_leds(self, leds):
        """Sets status of the leds according to the `leds` bitmask"""
        head, payload = self._transceive_with_retry(
            PacketId.SET_LED,
            bytes([leds]),
            ResponseLength.RESPONSE_ATR.value
        )
        if head.id != PacketId.ACK.value:
            # The Reader did something it shouldn't have done; reset him
            raise ReaderError()

    def beep(self, tones, repeat=False):
        if len(tones) > Reader.MAX_BEEP_LENGTH:
            raise ValueError("Cannot beep more than 8 times in one command")
        payload = bytearray()
        for tone in tones:
            payload += struct.pack(STRUCT_FORMAT + 'HH', tone[0], tone[1])
        payload += bytes([0x01 if repeat else 0x00])
        head, payload = self._transceive_with_retry(
            PacketId.BEEP,
            payload,
            ResponseLength.RESPONSE_ATR.value
        )
        if head.id != PacketId.ACK.value:
            # The Reader did something it shouldn't have done; reset him
            raise CorruptedPacketException()

    def RFID_send(self, payload):
        if len(payload) > Reader.MAX_RFID_PAYLOAD:
            raise ValueError("Cannot transmit more than 128 bytes at once")
        head, payload = self._transceive_with_retry(
            PacketId.RFID_SEND,
            payload,
            len(payload) + ResponseLength.TRANSMIT_OVERHEAD.value
        )
        if head.id != PacketId.RFID_SEND_COMPLETE.value:
            raise ReaderError()
        return payload
