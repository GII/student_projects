import json
import os
import logging

# from botocore.vendored import requests

import urllib3

# Initializing a logger and settign it to INFO

logger = logging.getLogger()
logger.setLevel(logging.INFO)

# Reading environment variables and generating a Telegram Bot API URL

# TOKEN = os.environ["TELEGRAM_TOKEN"]
TOKEN = "XXX"
# USER_ID = os.environ["CHAT_ID"]
USER_ID = "XXX"

TELEGRAM_URL = "https://api.telegram.org/bot{}/sendMessage".format(TOKEN)


# Main Lambda handler


def lambda_handler(event, context):

    # logging the event for debugging

    logger.info("event=")

    logger.info(json.dumps(event))

    # Basic exception handling. If anything goes wrong, logging the exception

    try:

        message = {
            "text": "Detección de intruso en {0} en fecha {1}, hora {2} y en {3}.".format(
                str(event["Raspberry_n"]), str(event["Fecha"]), str(event["Hora"]), str(event["Zona_n"])
            ),
            "chat_id": "XXX",
        }
        encoded_data = json.dumps(message).encode("utf-8")

        http = urllib3.PoolManager()
        resp = http.request("POST", TELEGRAM_URL, body=encoded_data, headers={"Content-Type": "application/json"})

        return resp.data

    except Exception as e:

        raise e
