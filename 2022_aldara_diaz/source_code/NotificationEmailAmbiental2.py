# CÓDIGO PARA LA NOTIFICACIÓN DE ERROR EN EL PARÁMETRO AMBIENTAL DE ILUMINANCIA

import boto3
from datetime import datetime
from datetime import time
import sys

client = boto3.client('ses')

def lambda_handler (event, context):
    inicio=time(9)
    final=time(21)
    if datetime.now().time()>inicio and datetime.now().time()<final:
        iluminancia=float(str(event['illuminance']))
        # Format text message from data
        if iluminancia<10000:
            message_text = "El valor de iluminancia es muy bajo. El valor es {0} lux." .format(
            str(event['illuminance']))
        elif iluminancia>40000:
            message_text = "El valor de iluminancia es muy alto. El valor es {0} lux." .format(
            str(event['illuminance']))
        client.send_email(
            Source = 'xxxxxxxx@correo1.com',    
            Destination = {
                'ToAddresses': ['xxxxxxxx@correo2.com']
            },
            Message = {
                'Subject': {
                    'Data': 'Amazon SES Notification',
                    'Charset': 'UTF-8'
                },
                'Body': {
                    'Text': {
                        'Data': message_text,                
                        'Charset': 'UTF-8'
                    }
                }
            })
    else:
        sys.exit()