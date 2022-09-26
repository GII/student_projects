# CÓDIGO PARA LA NOTIFICACIÓN DE ERROR EN LOS PARÁMETROS DEL TERRENO (pH Y TEMPERATURA)

import boto3
from datetime import datetime
from datetime import time
import sys

client = boto3.client('ses')

def lambda_handler (event, context):
    inicio=time(9)
    final=time(21)
    if datetime.now().time()>inicio and datetime.now().time()<final:
        ph=float(str(event['phSoil']))
        temperature=float(str(event['temperatureSoil']))

        # Format text message from data
        if ph<6 and 16<temperature<24:
            message_text = "El valor del pH es muy ácido, pH= {0}." .format(
            str(event['phSoil']))
        elif ph>7 and 16<temperature<24:
            message_text = "El valor del pH es muy básico, pH= {0}." .format(
            str(event['phSoil']))
            
        elif temperature<16 and 6<ph<7:
            message_text = "La temperatura en el suelo es muy baja, T= {0} ºC.". format(
            str(event['temperatureSoil']))
            
        elif temperature>24 and 6<ph<7:
            message_text = "La temperatura en el suelo es muy alta, T= {0} ºC.". format(
            str(event['temperatureSoil']))
        
        else:
            message_text = "Los parámetros del terreno se encuentran fuera del rango óptimo. El valor del pH es {0} y de la temperatura {1} ºC" .format(
                str(event['phSoil']), str(event['temperatureSoil']))
            
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