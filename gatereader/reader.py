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

class Reader:

    # GENERAL TODO: Properly handle CurruptedPacketException everywhere

    class ResetException(Exception):
        pass

    class CorruptedPacketException(Exception):
        pass

    class Leds(Enum):
        RED_LED     = 0
        GREEN_LED   = 1
        BLUE_LED    = 2

    BAUDRATE   = 19200
    BYTESIZE   = serial.EIGHTBITS
    PARITY     = serial.PARITY_EVEN
    STOPBITS   = serial.STOPBITS_ONE
    RX_TIMEOUT = 0.5

    RESPONSE_ATR    = 5
    PACKET_OVERHEAD = 3

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

    def _checksum_mismatch(self, head, payload, checksum):
        check =  head.id
        check ^= head.length
        for byte in payload:
            check ^= byte
        return check != checksum

    def _expect_packet(self, length):
        p, payload = self.PacketHead.unpack_from(self.port.read(length))
        payload, checksum = payload[:-1], payload[-1]
        if self._checksum_mismatch(p, payload, checksum):
            raise Reader.CorruptedPacketException()
        return p, payload

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
                # Now we are in serious trouble
                pass
            raise ResetException()

    def set_leds(self, leds):
        self._check_atr()
        self._transmit_packet(PacketId.SET_LED, bytes([leds]))
        try:
            head, payload = self._expect_packet(Reader.RESPONSE_ATR)
            if head.id != PacketId.ACK.value:
                raise Reader.CorruptedPacketException()
        except serial.SerialTimeoutException:
            # TODO do something about it
            pass

    def beep(self, tones, repeat=False):
        self._check_atr()
        payload = bytearray(0)
        for tone in tones:
            payload += struct.pack(STRUCT_FORMAT + 'H', tone[0])
            payload += struct.pack(STRUCT_FORMAT + 'H', tone[1])
        if repeat:
            payload += bytes([0x01])
        else:
            payload += bytes([0x00])
        self._transmit_packet(PacketId.BEEP, payload)
        try:
            head, payload = self._expect_packet(Reader.RESPONSE_ATR)
            if head.id != PacketId.ACK.value:
                raise Reader.CorruptedPacketException()
        except serial.SerialTimeoutException:
            # TODO do something about it
            pass

    def RFID_send(self, payload):
        # TODO check if payload is larger than 128 bytes
        self._check_atr()
        self._transmit_packet(PacketId.RFID_SEND, payload)
        try:
            head, payload = self._expect_packet(len(payload)
                                                + Reader.PACKET_OVERHEAD)
            if head.id != PacketId.RFID_SEND_COMPLETE.value:
                print(head.id)
                raise Reader.CorruptedPacketException()
            return payload
        except serial.SerialTimeoutException:
            # TODO do something about it
            pass
