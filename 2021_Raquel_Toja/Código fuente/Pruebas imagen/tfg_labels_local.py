# Análisis de una imagen cargada desde un sistema de archivos local (Labels)
# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# PDX-License-Identifier: MIT-0 (For details, see https://github.com/awsdocs/amazon-rekognition-developer-guide/blob/master/LICENSE-SAMPLECODE.)

import boto3


def detect_labels_local_file(photo):

    client = boto3.client("rekognition", aws_access_key_id="XXX", aws_secret_access_key="XXX", region_name="eu-west-1")

    with open("/home/pi/tfg/photo2.jpg", "rb") as image:
        response = client.detect_labels(Image={"Bytes": image.read()})

    print("Detected labels in " + photo)
    for label in response["Labels"]:
        print(label["Name"] + " : " + str(label["Confidence"]))

    return len(response["Labels"])


def main():
    photo = "photo"

    label_count = detect_labels_local_file(photo)
    print("Labels detected: " + str(label_count))


if __name__ == "__main__":
    main()
