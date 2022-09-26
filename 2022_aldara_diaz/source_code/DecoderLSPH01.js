# CÓDIGO PARA DECODIFICAR LOS MENSAJES ENVIADOS POR LSPH01

var AWS = require('aws-sdk');
console.log('Loading function');

exports.handler = (event, context, callback) => {
    var data = Buffer.from(event.PayloadData, 'base64');
    var chars = [...data];

    var params = Decoder(chars, event.WirelessMetadata.LoRaWAN.FPort);
    var iotdata = new AWS.IotData({ endpoint: 'xxxxxxxxxxxxx-xxx.iot.eu-west-1.amazonaws.com' });

    var response = {
        topic: "Parámetros_Terreno_Decoder",
        payload: JSON.stringify(params),
        qos: 0
    };

    iotdata.publish(response, function (err, data) {
        if (err) {
            console.log("ERROR => " + JSON.stringify(err));
        }
        else {
            console.log("publish data: ", response);
        }
    });

    callback(null, {
        statusCode: 200,
        body: JSON.stringify(params)
    });
};

function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var value = (bytes[0] << 8 | bytes[1]) & 0x3FFF;
    var batV = value / 1000;//Battery,units:V

    value = bytes[2] << 8 | bytes[3];
    if (bytes[2] & 0x80) { value |= 0xFFFF0000; }
    var temp_DS18B20 = (value / 10).toFixed(2);//DS18B20,temperature

    value = bytes[4] << 8 | bytes[5];
    var PH1 = (value / 100).toFixed(2);

    value = bytes[6] << 8 | bytes[7];
    var temp = 0;
    if ((value & 0x8000) >> 15 === 0)
        temp = (value / 10).toFixed(2);//temp_SOIL,temperature
    else if ((value & 0x8000) >> 15 === 1)
        temp = ((value - 0xFFFF) / 10).toFixed(2);

    var i_flag = bytes[8];
    var mes_type = bytes[10];
    return {
        Bat: batV,
        TempC_DS18B20: temp_DS18B20,
        phSoil: PH1,
        temperatureSoil: temp,
        Interrupt_flag: i_flag,
        Message_type: mes_type
    };
}
