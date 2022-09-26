# CÓDIGO PARA LA NOTIFICACIÓN DE ERROR EN LOS PARÁMETROS AMBIENTALES (TEMPERATURA Y HUMEDAD)

import boto3
from datetime import datetime
from datetime import time
import sys

client = boto3.client('ses')

def lambda_handler (event, context):
    inicio=time(9)
    final=time(21)
    if datetime.now().time()>inicio and datetime.now().time()<final:
        temperatura=float(str(event['temperature']))
        humedad=float(str(event['humidity']))

        # Format text message from data
        if temperatura<18 and 50<humedad<70:
            message_text = "La temperatura ambiental es muy baja, T= {0} ºC." .format(
            str(event['temperature']))
        elif temperatura>28 and 50<humedad<70:
            message_text = "La temperatura ambiental es muy alta, T= {0} ºC." .format(
            str(event['temperature']))
            
        elif humedad<50 and 18<temperatura<28:
            message_text = "La humedad ambiental es muy baja {0} %.". format(
            str(event['humidity']))
            
        elif humedad>70 and 18<temperatura<28:
            message_text = "La humedad ambiental es muy alta {0} %.". format(
            str(event['humidity']))
        
        else:
            message_text = "Los parámetros ambientales se encuentran fuera del rango óptimo. El valor del temperatura es {0} ºC y de la humedad {1} %" .format(
                str(event['temperature']), str(event['humidity']))
            
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