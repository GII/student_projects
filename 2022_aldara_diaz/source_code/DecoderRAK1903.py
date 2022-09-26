# CÓDIGO PARA DECODIFICAR LOS MENSAJES ENVIADOS POR RAK1903

import base64
import json
import logging
import ctypes
from tempfile import tempdir
import boto3

# define function name
FUNCTION_NAME = "FUNCTION NAME"

# Second Byte in Payload represents Data Types
# Low Power Payload Reference: https://developers.mydevices.com/cayenne/docs/lora/
DATA_TYPES = 1

# Type Illuminance
TYPE_ILUM= 0x65

# setup iot-data client for boto3
client = boto3.client('iot-data')

# setup logger
logger = logging.getLogger(FUNCTION_NAME)
logger.setLevel(logging.INFO)

def decode(event):
  data_base64 = event.get("PayloadData")
  data_decoded = base64.b64decode(data_base64)

  result = {
      "devEui": event.get("WirelessMetadata").get("LoRaWAN").get("DevEui"),
      "fPort": event.get("WirelessMetadata").get("LoRaWAN").get("FPort"),
      "freq": event.get("WirelessMetadata").get("LoRaWAN").get("Frequency"),
      "timestamp": event.get("WirelessMetadata").get("LoRaWAN").get("Timestamp")
  }

  if data_decoded[DATA_TYPES] == TYPE_ILUM:
      ilum = (data_decoded[DATA_TYPES + 1] << 8) | (data_decoded[DATA_TYPES + 2])
      ilum = ctypes.c_int16(ilum).value
      result['Illuminance'] = ilum

  return result

def lambda_handler(event, context):
  data = decode(event)
  logger.info("Data: %s" % json.dumps(data))

  response = client.publish(
      topic = "Parámetros_Ambientales_Decoder", qos = 0, payload = json.dumps(data)
  )

  return response