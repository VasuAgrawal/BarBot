import sys
import asyncio

HEADER_LEN = 4 # Number of bytes for the header
ENDIANNESS = "little"

# Protobuf strings are serialized to bytes encoded in UTF8
def make_packet_from_bytes(data_bytes):
    # Just do a length encoding of the data bytes
    try:
        # Unsigned integer
        header = len(data_bytes).to_bytes(HEADER_LEN, ENDIANNESS)
    except OverflowError as e:
        # The data is too long to fit, so we have to do something.
        raise e

    return header + data_bytes

@asyncio.coroutine
def read_packet_from_stream(stream):
    # Based on the header, read data from the stream and return the bytes of the
    # data packet.
    header = yield stream.read_bytes(HEADER_LEN)
    num_to_read = int.from_bytes(header, ENDIANNESS)
    data = yield stream.read_bytes(num_to_read)
    return data
