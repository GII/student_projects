# CÓDIGO PARA DECODIFICAR LOS MENSAJES ENVIADOS POR EM500 - CO2

var AWS = require('aws-sdk');
console.log('Loading function');

exports.handler = (event, context, callback) => {
    var data = Buffer.from(event.PayloadData, 'base64');
    var chars = [...data];

    var params = Decoder(chars, event.WirelessMetadata.LoRaWAN.FPort);
    var iotdata = new AWS.IotData({ endpoint: 'xxxxxxxxxxxxx-xxx.iot.eu-west-1.amazonaws.com' });

    var response = {
        topic: "Parámetros_Ambientales_Decoder",
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
    var decoded = {};

    for (var i = 0; i < bytes.length;) {
        var channel_id = bytes[i++];
        var channel_type = bytes[i++];
        // BATTERY
        if (channel_id === 0x01 && channel_type === 0x75) {
            decoded.battery = bytes[i];
            i += 1;
        }
        // TEMPERATURE
        else if (channel_id === 0x03 && channel_type === 0x67) {
            // ℃
            decoded.temperature = readInt16LE(bytes.slice(i, i + 2)) / 10;
            i += 2;


        }
        // HUMIDITY
        else if (channel_id === 0x04 && channel_type === 0x68) {
            decoded.humidity = bytes[i] / 2;
            i += 1;
        }
        // CO2
        else if (channel_id === 0x05 && channel_type === 0x7D) {
            decoded.co2 = readUInt16LE(bytes.slice(i, i + 2));
            i += 2;
        }
        // PRESSURE
        else if (channel_id === 0x06 && channel_type === 0x73) {
            decoded.pressure = readUInt16LE(bytes.slice(i, i + 2)) / 10;
            i += 2;
        } else {
            break;
        }
    }

    return decoded;
}

/* ******************************************
 * bytes to number
 ********************************************/
function readUInt16LE(bytes) {
    var value = (bytes[1] << 8) + bytes[0];
    return value & 0xffff;
}

function readInt16LE(bytes) {
    var ref = readUInt16LE(bytes);
    return ref > 0x7fff ? ref - 0x10000 : ref;
}