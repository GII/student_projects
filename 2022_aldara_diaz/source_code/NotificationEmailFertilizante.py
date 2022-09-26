# CÓDIGO PARA LA NOTIFICACIÓN DE ERROR EN EL NIVEL DE FERTILIZANTE

import boto3
client = boto3.client('ses')

def lambda_handler (event, context):
    # Format text message from data
    message_text = "El nivel de fertilizante es bajo. El valor es: {0} L" .format(str(event['Nivel_fertilizante']))
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
        }
)
